#include "end_condition.hpp"


EndCondition position_near(lib15442c::Vec position, double threshold)
{
    return [&position, &threshold](lib15442c::Pose pose)
    {
        return pose.vec().distance_to(position) < threshold;
    };
}

EndCondition heading_near(lib15442c::Angle heading, lib15442c::Angle threshold)
{
    return [&heading, &threshold](lib15442c::Pose pose)
    {
        return (heading - pose.angle) < threshold;
    };
}

EndCondition either(EndCondition a, EndCondition b)
{
    return [&a, &b](lib15442c::Pose pose)
    {
        return (a(pose) || b(pose));
    };
}

EndCondition both(EndCondition a, EndCondition b)
{
    return [&a, &b](lib15442c::Pose pose)
    {
        return (a(pose) && b(pose));
    };
}

bool default_end_condition(lib15442c::Pose)
{
    return false;
}