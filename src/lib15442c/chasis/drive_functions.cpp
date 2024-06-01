#include "drive_controller.hpp"
#include "lib15442c/math/math.hpp"
#include "lib15442c/logger.hpp"

#include <algorithm>

#define LOGGER "Drive Functions"

void lib15442c::DriveController::drive(double distance, DriveParameters parameters)
{
    #ifndef LIB15442C_MOCK_DEVICES_ONLY
    if (parameters.async)
    {
        parameters.async = false;

        pros::Task([this, distance, parameters]
                   { drive(distance, parameters); });
        pros::delay(10); // Give the task time to start
        return;
    }
    async_mutex.lock();
    #endif

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

        speed = std::min(fabs(speed), parameters.max_speed) * lib15442c::sgn(speed);
        speed = std::max(fabs(speed), parameters.min_speed) * lib15442c::sgn(speed);

        if (fabs(speed) < 40)
        {
            total_error += error;
        }

        speed += std::min(fabs(total_error) * 0.34, 30.0) * lib15442c::sgn(error);

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
    #ifndef LIB15442C_MOCK_DEVICES_ONLY
    async_mutex.unlock();
    #endif

    INFO_TEXT("End Drive!");
}

void lib15442c::DriveController::drive_time(double voltage, double time, DriveTimeParameters parameters) {
    double start_time = pros::millis();

    turn_pid->reset();

    while (pros::millis() - start_time < time) {
        double current_time = pros::millis() - start_time;

        double ramp_up_voltage = parameters.ramp_up ? parameters.ramp_speed * current_time : voltage;
        double ramp_down_voltage = parameters.ramp_down ? -parameters.ramp_speed * (current_time - time) : voltage;
        double drive_speed = std::min(ramp_up_voltage, std::min(voltage, ramp_down_voltage));

        double turn_speed = 0;

        if (!parameters.angle.is_none()) {
            Angle error = odometry->getRotation().error_from(parameters.angle);

            turn_speed = turn_pid->calculateError(error.deg());
        }

        drivetrain->move(drive_speed, turn_speed);

        pros::delay(10);
    }

    drivetrain->move(0, 0);
}