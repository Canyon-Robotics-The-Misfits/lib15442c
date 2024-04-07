#include "lib15442c/chasis/drive_controller.hpp"
#include "lib15442c/logger.hpp"
#include "lib15442c/math/math.hpp"

#define LOGGER "Boomerang"

void lib15442c::DriveController::boomerang(lib15442c::Pose target_pos, BoomerangParameters parameters)
{
    if (parameters.async)
    {
        parameters.async = false;
        
        pros::Task([this, target_pos, parameters]
                   { boomerang(target_pos, parameters); });
        pros::delay(10); // Give the task time to start
        return;
    }
    async_mutex.lock();
    
    INFO("Start boomerang... (%f, %f)", target_pos.x, target_pos.y);

    drive_pid->reset();
    turn_pid->reset();

    int startingTime = pros::millis();

    auto comp_status = pros::competition::get_status();

    Angle target_angle = target_pos.angle + (180_deg * parameters.backwards);

    Vec position = odometry->getPosition();
    bool initial_above_approach_line = position.y > tan(target_angle.rad()) * (position.x - target_pos.x) + target_pos.y;

    while (pros::competition::get_status() == comp_status)
    {
        Vec position = odometry->getPosition();
        double error = position.distance_to(target_pos.vec());

        Pose caret = target_pos;

        if (!target_angle.is_none())
        {
            caret -= pos(cos(-target_angle.rad() + M_PI / 2.0), sin(-target_angle.rad() + M_PI / 2.0)) * parameters.lead * error;
        }

        Angle carret_angle = position.angle_to(caret.vec()) + (180_deg * parameters.backwards);

        // If it is close to the end, focus on getting to the right angle and use cross track error
        if (error < parameters.cross_track_threshold && !target_angle.is_none())
        {
            carret_angle = target_angle;
            double newTargetX = odometry->getY() + caret.x * tan((odometry->getRotation() + 90_deg).rad()) - odometry->getX() * tan(odometry->getRotation().rad()) - caret.y;
            newTargetX /= tan((odometry->getRotation() + 90_deg).rad()) - tan(odometry->getRotation().rad());
            double newTargetY = odometry->getY() + (newTargetX - odometry->getX()) * tan(odometry->getRotation().rad());
            // double temp = error;
            error = pos(newTargetX, newTargetY).vec().distance_to(target_pos.vec());
            // std::cout << temp << ", " << error << std::endl;
        }

        bool above_approach_line = position.y > tan(target_angle.rad()) * (position.x - target_pos.x) + target_pos.y;
        if (parameters.chained && fabs(error) < parameters.chain_threshold && initial_above_approach_line != above_approach_line) {
            break;
        }
        else if (parameters.chained && fabs(error) < parameters.threshold)
        {
            break;
        }
        else if (!parameters.chained && fabs(error) < parameters.threshold)
        {
            break;
        }

        if ((int)pros::millis() - startingTime >= parameters.timeout)
        {
            WARN_TEXT("boomerang timed out!");
            break;
        }

        double speed = drive_pid->calculateError(error) * (parameters.backwards ? -1 : 1);

        // speed += std::fmin(fabs(totalError) * 0.34, 30.0) * lib15442c::sgn(error);

        Angle angle_error = odometry->getRotation().error_from(carret_angle);
        double rot_speed = -turn_pid->calculateError(angle_error.deg()) * (error < 2 ? error / 2.0 : 1.0); // NOTE: remove <2 /2 for testing

        speed = std::fmax(fabs(speed), 18) * lib15442c::sgn(speed);

        if (abs(speed) > parameters.max_speed)
        {
            speed = parameters.max_speed * lib15442c::sgn(speed);
        }

        if (abs(rot_speed) > 127)
        {
            rot_speed = 127 * lib15442c::sgn(rot_speed);
        }

        // Multiple the speed on a scale of 0-1 based on the angle error
        if (angle_error.deg() != 0 && parameters.turn_priority != -1)
        {
            speed *= fmax(fmin(fabs(parameters.turn_priority / fabs(angle_error.deg())), 1), 0);
        }

        // std::cout << odometry->getX() << ", " << odometry->getY() << ", " << caret.x << ", " << caret.y << ", " << odometry->getRotation() << ", " << angle_error << std::endl;

        drivetrain->move_ratio(speed, rot_speed);

        pros::delay(20);
    }

    if (!parameters.chained)
    {
        drivetrain->move(0, 0);
    }

    async_mutex.unlock();
    
    INFO_TEXT("End boomerang!");
}