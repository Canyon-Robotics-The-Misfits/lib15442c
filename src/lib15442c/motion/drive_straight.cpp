#include "motion.hpp"
#include "lib15442c/math/math.hpp"
#include "lib15442c/logger.hpp"

#define LOGGER "Drive Straight"

lib15442c::DriveStraight::DriveStraight(double target_distance, std::shared_ptr<lib15442c::PID> drive_pid, std::shared_ptr<lib15442c::PID> turn_pid, lib15442c::DriveParameters params)
    : target_distance(target_distance), drive_pid(drive_pid), turn_pid(turn_pid), params(params){};

bool lib15442c::DriveStraight::isAsync()
{
    return params.async;
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

    speed = std::clamp(fabs(speed), params.min_speed, params.max_speed) * lib15442c::sgn(speed);

    Angle angle_error = params.angle.error_from(pose.angle);
    double rot_speed = turn_pid->calculateError(angle_error.deg());

    // std::cout << pros::millis() << ", " << error << ", " << speed << std::endl;

    if (!params.chained && fabs(error) < params.threshold)
    {
        time_correct += delta_time;
    }
    else
    {
        time_correct = 0;
    }

    if (time_correct >= params.threshold_time || (params.chained && fabs(distance_traveled) >= fabs(target_distance) - params.threshold))
    {
        return MotionOutputExit {};
    }

    if (time_since_start >= params.threshold)
    {
        WARN_TEXT("drive timed out!");
        return MotionOutputExit {};
    }

    return MotionOutputSpeeds {
        linear_output : speed,
        rotational_output : rot_speed,
    };
}