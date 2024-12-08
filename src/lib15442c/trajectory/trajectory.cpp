#include "trajectory.hpp"

lib15442c::Trajectory::Trajectory(std::vector<TrajectoryState> states): states(states)
{
    
}

// TODO: make sure this actually works
lib15442c::TrajectoryState lib15442c::Trajectory::get_state(double time)
{
    double index = (states.size() - 1) / 2.0;
    double last_index = (states.size() - 1);

    while (floor(index) - floor(last_index) > 1)
    {
        double delta_index = (last_index - index) / 2.0;

        last_index = index;

        if (time > states[floor(index)].time)
        {
            index += abs(delta_index);
        }
        else if (time < states[floor(index)].time)
        {
            index -= abs(delta_index);
        }
        else
        {
            return states[floor(index)];
        }
    }

    int i = floor(index);
    if (time > states[i].time)
    {
        double t = (time - states[i].time) / (states[i+1].time - states[i].time);
        return lerp_state(states[i], states[i+1], t);
    }
    else
    {
        double t = (time - states[i-1].time) / (states[i].time - states[i-1].time);
        return lerp_state(states[i-1], states[i], t);
    }
}

double lib15442c::Trajectory::get_total_time()
{
    return states[states.size() - 1].time;
}