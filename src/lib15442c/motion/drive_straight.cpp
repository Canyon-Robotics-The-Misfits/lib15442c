#include "pid_motions.hpp"
#include "lib15442c/math/math.hpp"
#include "lib15442c/logger.hpp"

#define LOGGER "drive_straight.cpp"

lib15442c::DriveStraight::DriveStraight(double target_distance, std::shared_ptr<lib15442c::PID> drive_pid, std::shared_ptr<lib15442c::PID> turn_pid, lib15442c::DriveParameters params, std::string name)
    : target_distance(target_distance), drive_pid(drive_pid), turn_pid(turn_pid), params(params), name(name) {};

bool lib15442c::DriveStraight::isAsync()
{
    return params.async;
}

std::string lib15442c::DriveStraight::getName()
{
    return name;
}

void lib15442c::DriveStraight::initialize(Pose pose)
{
    drive_pid.reset();
    turn_pid.reset();

    time_correct = 0;
    starting_position = pose;
}

lib15442c::MotionOutput lib15442c::DriveStraight::calculate(Pose pose, double time_since_start, double delta_time)
{
    double distance_traveled = starting_position.vec().distance_to(pose.vec());
    double error = target_distance - (distance_traveled * lib15442c::sgn(target_distance));

    double speed = drive_pid->calculateError(error);

    // skeep speed between the min and max speed
    speed = std::clamp(fabs(speed), params.min_speed, params.max_speed) * lib15442c::sgn(speed);

    // Angle correction to drive straight
    Angle angle_error = params.angle.error_from(pose.angle);
    double rot_speed = turn_pid->calculateError(angle_error.deg());

    if (params.chained)
    {
        // if chained, exit after passing a line `params.threshold` inches in front of the target
        if (fabs(distance_traveled) >= fabs(target_distance) - params.threshold)
        {
            return MotionOutputExit{};
        }
    }
    else
    {
        // only exit if within the threshold for `params.threshold_time` ms
        if (!params.chained && fabs(error) < params.threshold)
        {
            time_correct += delta_time;
        }
        else
        {
            time_correct = 0;
        }

        if (time_correct >= params.threshold_time)
        {
            return MotionOutputExit{};
        }
    }

    if (time_since_start >= params.threshold)
    {
        WARN("\"%s\" timed out!", name.c_str());
        return MotionOutputExit{};
    }

    if (params.end_condition(pose))
    {
        WARN("\"%s\" reached end condition!", name.c_str());
        return MotionOutputExit{};
    }

    return MotionOutputSpeeds{
        linear_output : speed,
        rotational_output : rot_speed,
    };
}