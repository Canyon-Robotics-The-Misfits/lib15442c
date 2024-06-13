#include "drive_to.hpp"
#include "lib15442c/math/math.hpp"
#include "lib15442c/logger.hpp"

#define LOGGER "Boomerang"

lib15442c::Boomerang::Boomerang(Pose target_pose, std::shared_ptr<PID> drive_pid, std::shared_ptr<PID> turn_pid, BoomerangParameters params, std::string name)
    : target_pose(target_pose), drive_pid(drive_pid), turn_pid(turn_pid), params(params), name(name){};

bool lib15442c::Boomerang::isAsync()
{
    return params.async;
}

std::string lib15442c::Boomerang::getName()
{
    return name;
}

void lib15442c::Boomerang::initialize(Pose pose)
{
    drive_pid.reset();
    turn_pid.reset();

    initial_above_approach_line = pose.y > tan(target_pose.angle.rad()) * (pose.x - target_pose.x) + target_pose.y;
}

lib15442c::MotionOutput lib15442c::Boomerang::calculate(Pose pose, double time_since_start, double delta_time)
{
    double error = pose.vec().distance_to(target_pose.vec());

    Pose caret = target_pose;

    if (!target_pose.angle.is_none())
    {
        caret -= pos(cos(target_pose.angle.rad()), sin(target_pose.angle.rad())) * params.lead * error;
    }

    Angle target_angle = pose.vec().angle_to(caret.vec()) + (180_deg * params.backwards);

    // If it is close to the end, focus on getting to the right angle and use cross track error
    if (error < params.angle_priority_threshold && !target_pose.angle.is_none())
    {
        target_angle = target_pose.angle;
    }

    bool above_approach_line = pose.y > tan(target_pose.angle.rad()) * (pose.x - target_pose.x) + target_pose.y;
    if (params.chained && fabs(error) < params.chain_threshold && initial_above_approach_line != above_approach_line)
    {
        return MotionOutputExit {};
    }
    else if (params.chained && fabs(error) < params.threshold)
    {
        return MotionOutputExit {};
    }
    else if (!params.chained && fabs(error) < params.threshold)
    {
        return MotionOutputExit {};
    }

    if (time_since_start >= params.timeout)
    {
        WARN_TEXT("boomerang timed out!");
        return MotionOutputExit {};
    }

    double drive_speed = drive_pid->calculateError(error) * (params.backwards ? -1 : 1);

    if (abs(drive_speed) > params.max_speed)
    {
        drive_speed = params.max_speed * lib15442c::sgn(drive_speed);
    }

    if (abs(drive_speed) < params.min_speed)
    {
        drive_speed = params.min_speed * lib15442c::sgn(drive_speed);
    }

    Angle angle_error = pose.angle.error_from(target_angle);
    double rot_speed = -turn_pid->calculateError(angle_error.deg());

    if (abs(rot_speed) > 127)
    {
        rot_speed = 127 * lib15442c::sgn(rot_speed);
    }

    return MotionOutputSpeeds {
        linear_output : drive_speed,
        rotational_output : rot_speed,
    };
}