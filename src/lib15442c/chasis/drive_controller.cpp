#include "drive_controller.hpp"
#include "lib15442c/math/math.hpp"
#include "lib15442c/logger.hpp"

#define LOGGER "Drive Controller"

lib15442c::DriveController::DriveController(
            std::shared_ptr<IDrivetrain> drivetrain,
            std::shared_ptr<IOdometry> odometry,
            std::shared_ptr<PID> drive_pid, std::shared_ptr<PID> turn_pid,
            double default_min_speed) : drivetrain(drivetrain), odometry(odometry), drive_pid(drive_pid), turn_pid(turn_pid), default_min_speed(default_min_speed){};

void lib15442c::DriveController::turn(lib15442c::Angle angle, AngleParameters parameters)
{
    INFO("Start turn... (%f deg)", angle.deg());

    Angle global_angle = odometry->getRotation() + angle;

    face(FaceAngle { angle: global_angle }, parameters);
    
    INFO_TEXT("End turn!");
}

void lib15442c::DriveController::faceAngle(lib15442c::Angle angle, AngleParameters parameters)
{
    INFO("Start face angle... (%f deg)", angle.deg());

    face(FaceAngle { angle });
    
    INFO_TEXT("End face angle!");
}

void lib15442c::DriveController::facePoint(lib15442c::Pose point, lib15442c::Angle angle_offset, AngleParameters parameters)
{
    INFO("Start face point... (%f, %f)", point.x, point.y);

    face(FacePoint { point, angle_offset }, parameters);
    
    INFO_TEXT("End face point!");
}

void lib15442c::DriveController::face(FaceTarget target, AngleParameters parameters) {
    if (parameters.async)
    {
        parameters.async = false;

        pros::Task([this, target, parameters]
                   { face(target, parameters); });
        pros::delay(10); // Give the task time to start
        return;
    }
    async_mutex.lock();

    Angle target_angle;

    // Calculate target angle if facing a point
    if (FacePoint *point_target = std::get_if<FacePoint>(&target))
    {
        target_angle = odometry->getPosition().angle_to(point_target->pos.vec());
        if (point_target->pos.y - odometry->getY() < 0)
        {
            target_angle += 180_deg;
        }
        target_angle += point_target->angle_offset;
    } else {
        FaceAngle *angle_target = std::get_if<FaceAngle>(&target);

        target_angle = angle_target->angle;
    }

    // Calculate ratios
    Angle start_rotation = odometry->getRotation();
    Angle initial_error = start_rotation.error_from(target_angle);

    double left_arc_distance = (parameters.arc_radius + drivetrain->get_track_width() / 2.0) * initial_error.rad();
    double right_arc_distance = (parameters.arc_radius - drivetrain->get_track_width() / 2.0) * initial_error.rad();

    double left_ratio = lib15442c::sgn(left_arc_distance) * fabs(fmax(fmin(left_arc_distance / right_arc_distance, 1), -1));
    double right_ratio = lib15442c::sgn(right_arc_distance) * fabs(fmax(fmin(right_arc_distance / left_arc_distance, 1), -1));

    if (parameters.arc_radius == 0)
    {
        left_ratio = 1;
        right_ratio = -1;
    }

    turn_pid->reset();
    int time_correct = 0;

    int starting_time = pros::millis();

    auto comp_status = pros::competition::get_status();

    while (pros::competition::get_status() == comp_status)
    {
        // Calculate target angle if facing a point
        if (FacePoint *point_target = std::get_if<FacePoint>(&target))
        {
            target_angle = odometry->getPosition().angle_to(point_target->pos.vec());
            if (point_target->pos.y - odometry->getY() < 0)
            {
                target_angle += 180_deg;
            }
            target_angle += point_target->angle_offset;
        }

        Angle rotation = odometry->getRotation();
        Angle error = rotation.error_from(target_angle);

        if (fabs(error.deg()) < parameters.threshold.deg())
        {
            time_correct += 20;
        }
        else
        {
            time_correct = 0;
        }

        bool chainCondition = parameters.chained && (sgn(error.deg()) != sgn(initial_error.deg()) || fabs(error.deg()) < 10);
        if (time_correct >= 100 || chainCondition)
        {
            // std::cout << pros::millis() << ", " << rotation << ", " << targetAngle << ", " << error << std::endl;
            break;
        }

        if ((int)pros::millis() - starting_time >= parameters.timeout)
        {
            WARN_TEXT("Face timed out!");
            break;
        }

        double rot_speed = turn_pid->calculateError(error.deg());

        rot_speed = std::max(parameters.min_speed, (double)fabs(rot_speed)) * lib15442c::sgn(rot_speed);
        rot_speed = std::min(parameters.max_speed, (double)fabs(rot_speed)) * lib15442c::sgn(rot_speed);
        if (fabs(error.deg()) < 2.5)
        {
            rot_speed = parameters.min_speed * lib15442c::sgn(error.deg());
        }

        // std::cout << pros::millis() << ", " << rot_speed << ", " << error << std::endl;

        double leftSpeed = rot_speed * left_ratio;
        double rightSpeed = rot_speed * right_ratio;

        drivetrain->tank(leftSpeed, rightSpeed);

        pros::delay(20);
    }

    if (!parameters.chained)
    {
        drivetrain->move(0, 0);
    }
    async_mutex.unlock();
}

void lib15442c::DriveController::drive(double distance, DriveParameters parameters)
{
    if (parameters.async)
    {
        parameters.async = false;

        pros::Task([this, distance, parameters]
                   { drive(distance, parameters); });
        pros::delay(10); // Give the task time to start
        return;
    }
    async_mutex.lock();

    INFO("Start Drive... (%f in)", distance);

    drive_pid->reset();
    turn_pid->reset();

    Pose starting_position = odometry->getPose();

    double time_correct = 0;
    double total_error = 0;

    int starting_time = pros::millis();

    Angle target_rotation;
    if (parameters.angle.is_none())
    {
        target_rotation = odometry->getRotation();
    } else {
        target_rotation = parameters.angle;
    }

    auto comp_status = pros::competition::get_status();

    while (pros::competition::get_status() == comp_status)
    {
        // double left = left_motors->get_position() - left_start;
        // double right = right_motors->get_position() - right_start;
        // double parallel = parallel_tracker->get_position() / 100.0;
        // double parallelDistance = (parallel - starting_parallel) * degrees_per_inch;

        double distanceTraveled = starting_position.vec().distance_to(odometry->getPosition());
        double error = distance - (distanceTraveled * lib15442c::sgn(distance));

        double speed = drive_pid->calculateError(error);

        speed = fmin(fabs(speed), parameters.max_speed) * lib15442c::sgn(speed);
        speed = fmax(fabs(speed), parameters.min_speed) * lib15442c::sgn(speed);

        if (fabs(speed) < 40)
        {
            total_error += error;
        }

        speed += std::fmin(fabs(total_error) * 0.34, 30.0) * lib15442c::sgn(error);

        Angle angle_error = target_rotation.error_from(odometry->getRotation());
        double rot_speed = turn_pid->calculateError(angle_error.deg());

        // std::cout << pros::millis() << ", " << error << ", " << speed << std::endl;

        drivetrain->move(speed, rot_speed);

        if (!parameters.chained && fabs(error) < parameters.threshold)
        {
            time_correct += 20;
            break;
        }
        else
        {
            time_correct = 0;
        }

        if (time_correct >= 40 || (parameters.chained && fabs(distanceTraveled) >= fabs(distance) - parameters.threshold))
        {
            break;
        }

        if ((int)pros::millis() - starting_time >= parameters.threshold)
        {
            WARN_TEXT("drive timed out!");
            break;
        }

        pros::delay(20);
    }

    if (!parameters.chained)
    {
        drivetrain->move(0, 0);
    }
    async_mutex.unlock();

    INFO_TEXT("End Drive!");
}

bool lib15442c::DriveController::isDone(int timeout)
{
    bool done = async_mutex.try_lock_for(std::chrono::milliseconds(timeout));
    if (done)
    {
        async_mutex.unlock();
    }

    return done;
}

void lib15442c::DriveController::awaitDone()
{
    async_mutex.take(TIMEOUT_MAX);
    async_mutex.unlock();
}

bool lib15442c::DriveController::awaitNear(lib15442c::Vec pos, double distance)
{
    while (true)
    {
        if (odometry->getPosition().distance_to(pos) < distance)
        {
            return true;
        }

        if (isDone(5))
        {
            return false;
        }
    }
}

bool lib15442c::DriveController::awaitAngle(Angle angle, Angle threshold)
{
    while (true)
    {
        Angle rotation = odometry->getRotation();

        if (fabs(rotation.error_from(angle).deg()) <= threshold.deg())
        {
            return true;
        }

        if (isDone(5))
        {
            return false;
        }
    }
}