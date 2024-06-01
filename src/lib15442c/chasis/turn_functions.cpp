#include "drive_controller.hpp"
#include "lib15442c/math/math.hpp"
#include "lib15442c/logger.hpp"

#include <algorithm>

#define LOGGER "Turn Functions"

void lib15442c::DriveController::turn(lib15442c::Angle angle, AngleParameters parameters)
{
    INFO("Start turn... (%f deg)", angle.deg());

    Angle global_angle = odometry->getRotation() + angle;

    face(FaceAngleTarget { angle: global_angle }, parameters, false);
    
    INFO_TEXT("End turn!");
}

void lib15442c::DriveController::faceAngle(lib15442c::Angle angle, AngleParameters parameters)
{
    INFO("Start face angle... (%f deg)", angle.deg());

    face(FaceAngleTarget { angle }, parameters, false);
    
    INFO_TEXT("End face angle!");
}

void lib15442c::DriveController::facePoint(lib15442c::Pose point, lib15442c::Angle angle_offset, AngleParameters parameters)
{
    INFO("Start face point... (%f, %f)", point.x, point.y);

    face(FacePointTarget { point, angle_offset }, parameters, false);
    
    INFO_TEXT("End face point!");
}

void lib15442c::DriveController::face(FaceTarget target, AngleParameters parameters, bool log_ends) {
    if (log_ends) {
        
        if (FacePointTarget *point_target = std::get_if<FacePointTarget>(&target))
        {
            INFO("Start face... (%f, %f)", point_target->pos.x,point_target->pos.y);
        } else {
            FaceAngleTarget *angle_target = std::get_if<FaceAngleTarget>(&target);

            INFO("Start face... (%f deg)", angle_target->angle.deg());
        }
    }

    #ifndef LIB15442C_MOCK_DEVICES_ONLY
    if (parameters.async)
    {
        parameters.async = false;

        pros::Task([this, target, parameters]
                   { face(target, parameters); });
        pros::delay(10); // Give the task time to start
        return;
    }
    async_mutex.lock();
    #endif

    Angle target_angle;

    // Calculate target angle if facing a point
    if (FacePointTarget *point_target = std::get_if<FacePointTarget>(&target))
    {
        target_angle = odometry->getPosition().angle_to(point_target->pos.vec());
        if (point_target->pos.y - odometry->getY() < 0)
        {
            target_angle += 180_deg;
        }
        target_angle += point_target->angle_offset;
    } else {
        FaceAngleTarget *angle_target = std::get_if<FaceAngleTarget>(&target);

        target_angle = angle_target->angle;
    }

    // Calculate ratios
    Angle start_rotation = odometry->getRotation();
    Angle initial_error = start_rotation.error_from(target_angle);

    double left_arc_distance = (parameters.arc_radius + drivetrain->get_track_width() / 2.0) * initial_error.rad();
    double right_arc_distance = (parameters.arc_radius - drivetrain->get_track_width() / 2.0) * initial_error.rad();

    double left_ratio = lib15442c::sgn(left_arc_distance) * fabs(std::max(std::min(left_arc_distance / right_arc_distance, 1.0), -1.0));
    double right_ratio = lib15442c::sgn(right_arc_distance) * fabs(std::max(std::min(right_arc_distance / left_arc_distance, 1.0), -1.0));

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
        if (FacePointTarget *point_target = std::get_if<FacePointTarget>(&target))
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
    #ifndef LIB15442C_MOCK_DEVICES_ONLY
    async_mutex.unlock();
    #endif

    if (log_ends) {
        INFO_TEXT("End face!");
    }
}