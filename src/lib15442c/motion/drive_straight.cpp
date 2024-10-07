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

void lib15442c::DriveStraight::initialize(std::shared_ptr<IDrivetrain> drivetrain, Pose pose)
{
    drive_pid->reset_pid();
    turn_pid->reset_pid();

    time_correct = 0;
    starting_position = lib15442c::Pose::none();
}

lib15442c::MotionOutput lib15442c::DriveStraight::calculate(Pose pose, double time_since_start, double delta_time)
{
    if (starting_position.is_none())
    {
        starting_position = pose;
    }

    double distance_traveled = starting_position.vec().distance_to(pose.vec());
    double error = target_distance - (distance_traveled * lib15442c::sgn(target_distance));

    double speed = drive_pid->calculateError(error);

    // keep speed between the min and max speed
    speed = std::clamp(fabs(speed), params.min_speed, params.max_speed) * lib15442c::sgn(error);

    // Angle correction to drive straight
    if (params.angle.is_none())
    {
        params.angle = pose.angle;
    }

    Angle angle_error = pose.angle.error_from(params.angle);
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

    if (time_since_start >= params.timeout)
    {
        WARN("\"%s\" timed out!", name.c_str());
        return MotionOutputExit{};
    }

    if (params.end_condition(pose))
    {
        WARN("\"%s\" reached end condition!", name.c_str());
        return MotionOutputExit{};
    }

    // std::cout << time_since_start << ", " << error << ", " << speed << ", " << pose.angle.rad() << ", " << params.angle.rad() << ", " << angle_error.rad() << std::endl;

    return MotionOutputSpeeds{
        linear_output : speed,
        rotational_output : rot_speed,
    };
}