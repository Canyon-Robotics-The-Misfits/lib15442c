#include "drive_to.hpp"
#include "lib15442c/math/math.hpp"
#include "lib15442c/logger.hpp"

#define LOGGER "drive_to.cpp"

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

void lib15442c::Boomerang::initialize(std::shared_ptr<IDrivetrain> drivetrain, Pose pose)
{
    drive_pid->reset_pid();
    turn_pid->reset_pid();

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
        return MotionOutputExit{};
    }
    else if (params.chained && fabs(error) < params.threshold)
    {
        return MotionOutputExit{};
    }
    else if (!params.chained && fabs(error) < params.threshold)
    {
        return MotionOutputExit{};
    }

    if (time_since_start >= params.timeout)
    {
        WARN_TEXT("boomerang timed out!");
        return MotionOutputExit{};
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
    double rot_speed = turn_pid->calculateError(angle_error.deg()); // TODO: check if removed - breaks anything

    if (abs(rot_speed) > 127)
    {
        rot_speed = 127 * lib15442c::sgn(rot_speed);
    }

    return MotionOutputSpeeds{
        linear_output : drive_speed,
        rotational_output : rot_speed,
    };
}

lib15442c::DriveToAB::DriveToAB(Pose target_pose, std::shared_ptr<PID> drive_pid, DriveToABParameters params, std::string name)
    : target_pose(target_pose), drive_pid(drive_pid), params(params), name(name){};

bool lib15442c::DriveToAB::isAsync()
{
    return params.async;
}

std::string lib15442c::DriveToAB::getName()
{
    return name;
}

void lib15442c::DriveToAB::initialize(std::shared_ptr<IDrivetrain> drivetrain, Pose pose)
{
    drive_pid->reset_pid();
}

lib15442c::MotionOutput lib15442c::DriveToAB::calculate(Pose pose, double time_since_start, double delta_time)
{
    float distance = pose.vec().distance_to(target_pose.vec());
    float linear_velocity = drive_pid->calculateError(distance) * (params.backwards ? -1 : 1);

    Angle robot_angle = pose.angle + (params.backwards ? 180_deg : 0_deg);

    Angle alpha = Angle::from_rad(atan2(target_pose.y - pose.y, target_pose.x - pose.x) - robot_angle.rad());

    Angle beta = target_pose.angle - robot_angle - alpha;

    float angular_velocity = params.kp_alpha * alpha.rad() - params.kp_beta * beta.rad();

    if (abs(linear_velocity) > params.max_speed)
    {
        linear_velocity = params.max_speed * lib15442c::sgn(linear_velocity);
    }

    if (abs(linear_velocity) < params.min_speed)
    {
        linear_velocity = params.min_speed * lib15442c::sgn(linear_velocity);
    }

    if (time_since_start > params.timeout)
    {
        return MotionOutputExit{};
    }

    if (distance < params.threshold)
    {
        return MotionOutputExit{};
    }

    if (params.end_condition(pose))
    {
        WARN("\"%s\" reached end condition!", name.c_str());
        return MotionOutputExit{};
    }

    return MotionOutputSpeeds{
        linear_output : linear_velocity,
        rotational_output : angular_velocity
    };
}

lib15442c::DriveToIntermediate::DriveToIntermediate(Pose target_pose, std::shared_ptr<PID> drive_pid, std::shared_ptr<PID> turn_pid, DriveToIntermediateParameters params, std::string name)
    : target_pose(target_pose), drive_pid(drive_pid), turn_pid(turn_pid), params(params), name(name){};

bool lib15442c::DriveToIntermediate::isAsync()
{
    return params.async;
}

std::string lib15442c::DriveToIntermediate::getName()
{
    return name;
}

void lib15442c::DriveToIntermediate::initialize(std::shared_ptr<IDrivetrain> drivetrain, Pose pose)
{
    drive_pid->reset_pid();
}

lib15442c::MotionOutput lib15442c::DriveToIntermediate::calculate(Pose pose, double time_since_start, double delta_time)
{
    pose.angle += params.backwards ? 180_deg : 0_deg;

    Angle absTargetAngle = pose.vec().angle_to(target_pose.vec());

    float distance = pose.vec().distance_to(target_pose.vec());
    Angle alpha = absTargetAngle - target_pose.angle;
    Angle errorTerm1 = absTargetAngle - pose.angle;

    Angle beta = Angle::from_rad(atan(params.r / distance));

    if (alpha.deg() < 0)
    {
        beta = Angle::from_deg(-beta.deg());
    }

    Angle turnError;
    if (fabs(alpha.deg()) < fabs(beta.deg()))
    {
        turnError = errorTerm1 + alpha;
    }
    else
    {
        turnError = errorTerm1 + beta;
    }

    if (distance < params.settle_threshold)
    {
        distance = distance * sgn(cos(turnError.rad()));
        turnError = target_pose.angle - pose.angle;
    }

    float linear_velocity = drive_pid->calculateError(distance);
    float angular_velocity = turn_pid->calculateError(turnError.deg());

    if (abs(linear_velocity) > params.max_speed)
    {
        linear_velocity = params.max_speed * lib15442c::sgn(linear_velocity);
    }

    if (abs(linear_velocity) < params.min_speed)
    {
        linear_velocity = params.min_speed * lib15442c::sgn(linear_velocity);
    }

    if (time_since_start > params.timeout)
    {
        return MotionOutputExit{};
    }

    if (distance < params.threshold)
    {
        return MotionOutputExit{};
    }

    return MotionOutputSpeeds{
        linear_output : linear_velocity,
        rotational_output : angular_velocity
    };
}