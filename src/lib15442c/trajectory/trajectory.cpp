#include <iostream>

#include "trajectory.hpp"
#include "lib15442c/math/vector.hpp"

#include <cmath>

lib15442c::TrajectoryState lib15442c::Trajectory::lerp_state(TrajectoryState a, TrajectoryState b, double t)
{
    return TrajectoryState {
        position: lib15442c::Vec(
            std::lerp(a.position.x, b.position.x, t),
            std::lerp(a.position.y, b.position.y, t)
        ),
        drive_velocity: std::lerp(a.drive_velocity, b.drive_velocity, t),
        rotational_velocity: std::lerp(a.rotational_velocity, b.rotational_velocity, t),
        time: std::lerp(a.time, b.time, t),
        heading: (b.heading - a.heading) * t + a.heading
    };
}

lib15442c::Trajectory::Trajectory(std::vector<TrajectoryState> states): states(states)
{
    
}

// TODO: make sure this actually works
lib15442c::TrajectoryState lib15442c::Trajectory::get_state(double time)
{
    int index = -1;
    for (int i = 1; i < (int)states.size(); i++)
    {
        if (states[i].time >= time)
        {
            index = i;
            break;
        }
    }

    if (index == -1)
    {
        index = states.size() - 1;
    }

    double t = (time - states[index-1].time) / (states[index].time - states[index-1].time);
    return lerp_state(states[index-1], states[index], t);
}

double lib15442c::Trajectory::get_total_time()
{
    return states[states.size() - 1].time;
}

void lib15442c::Trajectory::debug_log()
{
    for (TrajectoryState state : states)
    {
        std::cout << state.time << ", " << state.position.x << ", " << state.position.y << ", " << state.heading.rad() << ", " << state.drive_velocity << ", " << state.rotational_velocity << std::endl;
    }
}