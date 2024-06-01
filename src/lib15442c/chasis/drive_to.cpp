#include "lib15442c/chasis/drive_controller.hpp"
#include "lib15442c/logger.hpp"
#include "lib15442c/math/math.hpp"

#define LOGGER "Boomerang"


void lib15442c::DriveController::boomerang(lib15442c::Pose target_pos, BoomerangParameters parameters)
{
    #ifndef LIB15442C_MOCK_DEVICES_ONLY
    if (parameters.async)
    {
        parameters.async = false;
        
        pros::Task([this, target_pos, parameters]
                   { boomerang(target_pos, parameters); });
        pros::delay(10); // Give the task time to start
        return;
    }
    async_mutex.lock();
    #endif
    
    INFO("Start boomerang... (%f, %f)", target_pos.x, target_pos.y);

    drive_pid->reset();
    turn_pid->reset();

    int startingTime = pros::millis();

    auto initial_comp_status = pros::competition::get_status();

    Angle end_angle = target_pos.angle + (180_deg * parameters.backwards);

    Vec position = odometry->getPosition();
    bool initial_above_approach_line = position.y > tan(end_angle.rad()) * (position.x - target_pos.x) + target_pos.y;

    while (pros::competition::get_status() == initial_comp_status)
    {
        Vec position = odometry->getPosition();
        double error = position.distance_to(target_pos.vec());

        Pose caret = target_pos;

        if (!end_angle.is_none())
        {
            caret -= pos(cos(end_angle.rad()), sin(end_angle.rad())) * parameters.lead * error;
        }

        Angle target_angle = position.angle_to(caret.vec()) + (180_deg * parameters.backwards);

        // If it is close to the end, focus on getting to the right angle and use cross track error
        if (error < parameters.angle_priority_threshold && !end_angle.is_none())
        {
            target_angle = end_angle;
        }

        bool above_approach_line = position.y > tan(end_angle.rad()) * (position.x - target_pos.x) + target_pos.y;
        if (parameters.chained && fabs(error) < parameters.chain_threshold && initial_above_approach_line != above_approach_line)
        {
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

        double drive_speed = drive_pid->calculateError(error) * (parameters.backwards ? -1 : 1);

        if (abs(drive_speed) > parameters.max_speed)
        {
            drive_speed = parameters.max_speed * lib15442c::sgn(drive_speed);
        }
        
        if (abs(drive_speed) < parameters.min_speed)
        {
            drive_speed = parameters.min_speed * lib15442c::sgn(drive_speed);
        }
        
        Angle angle_error = odometry->getRotation().error_from(target_angle);
        double rot_speed = -turn_pid->calculateError(angle_error.deg());

        if (abs(rot_speed) > 127)
        {
            rot_speed = 127 * lib15442c::sgn(rot_speed);
        }

        // std::cout << position.x << ", " << position.y << ", " << caret.x << ", " << caret.y << ", " << std::endl;

        drivetrain->move_ratio(drive_speed, rot_speed);

        pros::delay(20);
    }

    if (!parameters.chained)
    {
        drivetrain->move(0, 0);
    }

    #ifndef LIB15442C_MOCK_DEVICES_ONLY
    async_mutex.unlock();
    #endif
    
    INFO_TEXT("End boomerang!");
}