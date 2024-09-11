#include "pid_motions.hpp"
#include "lib15442c/math/math.hpp"
#include "lib15442c/logger.hpp"

#define LOGGER "face.cpp"

lib15442c::Face::Face(FaceTarget target, std::shared_ptr<PID> pid, FaceParameters params, std::string name)
    : target(target), pid(pid), params(params), name(name) {};

bool lib15442c::Face::isAsync()
{
    return params.async;
}

std::string lib15442c::Face::getName()
{
    return name;
}

void lib15442c::Face::initialize(Pose pose)
{
    Angle target_angle = getTargetAngle(target, pose);

    initial_error = pose.angle.error_from(target_angle);
}

lib15442c::Angle lib15442c::Face::getTargetAngle(FaceTarget target, Pose pose)
{
    // Check which varient the target is, and find the target angle accordingly
    if (FacePointTarget *point_target = std::get_if<FacePointTarget>(&target))
    {
        return pose.vec().angle_to(point_target->pos.vec()) + point_target->angle_offset;
    }
    else
    {
        FaceAngleTarget *angle_target = std::get_if<FaceAngleTarget>(&target);

        return angle_target->angle;
    }
}

lib15442c::MotionOutput lib15442c::Face::calculate(Pose pose, double time_since_start, double delta_time)
{

    // Calculate target angle if facing a point
    Angle target_angle = getTargetAngle(target, pose);

    Angle error = pose.angle.error_from(target_angle);

    if (params.chained)
    {
        // if the error crossed 0 (passed target angle) or is within the threshold exit
        if (sgn(error.deg()) != sgn(initial_error.deg()) || fabs(error.deg() < params.threshold.deg()))
        {
            return MotionOutputExit{};
        }
    }
    else
    {
        // Must be within the threshold for `params.threshold_time` ms to exit
        if (fabs(error.deg()) < params.threshold.deg())
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

    double rot_speed = pid->calculateError(error.deg());

    // keep rot_speed between the min and max speeds
    rot_speed = std::clamp(fabs(rot_speed), params.min_speed, params.max_speed) * lib15442c::sgn(rot_speed);

    return MotionOutputSpeeds{
        linear_output : 0,
        rotational_output : rot_speed,
    };
}