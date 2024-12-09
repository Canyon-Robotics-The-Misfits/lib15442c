#include "trajectory_builder.hpp"

#include <iostream>

lib15442c::Zone lib15442c::circle_zone(lib15442c::Vec center, double radius, double value)
{
    
    return [center, radius, value](lib15442c::Vec point)
    {
        if (pow(point.x - center.x, 2) + pow(point.y - center.y, 2) < radius * radius)
        {
            return value;
        }
        else
        {
            return infinity();
        }
    };
}
lib15442c::Zone lib15442c::rect_zone(lib15442c::Vec corner_a, lib15442c::Vec corner_b, double value)
{
    return [corner_a, corner_b, value](lib15442c::Vec point)
    {
        if ((point - corner_a).x < corner_b.x && (point - corner_a).y < corner_b.y)
        {
            return value;
        }
        else
        {
            return infinity();
        }
    };
}



lib15442c::Vec lib15442c::TrajectoryBuilder::lerp_hermite(double t, Vec p0, Vec m0, Vec p1, Vec m1)
{
    return
        p0 * (2 * pow(t, 3) - 3 * pow(t, 2) + 1) +
        m0 * (pow(t, 3) - 2 * pow(t, 2) + t) + 
        p1 * (-2 * pow(t, 3) + 3 * pow(t, 2)) +
        m1 * (pow(t, 3) - pow(t, 2));
}

std::vector<lib15442c::TrajectoryState> lib15442c::TrajectoryBuilder::calculate_hermite(double resolution, Vec p0, Vec m0, Vec p1, Vec m1)
{
    std::vector<TrajectoryState> states;

    double t = 0.0;
    while (t < 1.0)
    {
        double dt = 0.05;
        t += dt;

        if (t > 1.0 - 0.001)
        {
            t = 1.0;
        }

        Vec next = lerp_hermite(t, p0, m0, p1, m1);
        
        states.push_back({
            position: next,
            drive_velocity: INFINITY,
            turn_velocity: INFINITY,
            time: INFINITY
        });
    }

    return states;
}

double lib15442c::TrajectoryBuilder::get_max_speed(Vec position)
{
    double min_value = INFINITY;
    for (lib15442c::Zone zone : max_speed_zones)
    {
        double value = zone(position);

        // std::cout << value << std::endl;

        if (value < min_value)
        {
            min_value = value;
        }
    }

    return min_value;
}

double lib15442c::TrajectoryBuilder::calculate_velocity(TrajectoryState final, TrajectoryState initial, double global_max_speed, double max_acceleration)
{
    double velocity_initial = initial.drive_velocity;
    double distance = initial.position.distance_to(final.position);

    double delta_time = (-velocity_initial + sqrt(velocity_initial * velocity_initial + 2 * distance * max_acceleration)) / max_acceleration;

    double max_speed = std::min(global_max_speed, get_max_speed(final.position));

    // std::cout << "max speed: " << max_speed << std::endl;

    return std::min(velocity_initial + max_acceleration * delta_time, max_speed);
}

lib15442c::TrajectoryBuilder::TrajectoryBuilder(HermitePair initial_pair)
{
    append_hermite(initial_pair);
}

void lib15442c::TrajectoryBuilder::append_hermite(HermitePair pair)
{
    hermite_spline.push_back(pair);
}

void lib15442c::TrajectoryBuilder::add_max_speed_zone(Zone zone)
{
    max_speed_zones.push_back(zone);
}

lib15442c::Trajectory lib15442c::TrajectoryBuilder::compute(TrajectoryConstraints constraints, double resolution)
{
    std::vector<TrajectoryState> states;

    states.push_back({
        position: hermite_spline[0].point,
        drive_velocity: INFINITY,
        turn_velocity: INFINITY,
        time: INFINITY
    });

    for (int i = 1; i < (int)hermite_spline.size(); i++)
    {

        std::vector<TrajectoryState> segment = 
            calculate_hermite(resolution, hermite_spline[i-1].point, hermite_spline[i-1].tangent, hermite_spline[i].point, hermite_spline[i].tangent);

        states.insert(states.end(), segment.begin(), segment.end());
    }

    states[0].drive_velocity = 0;
    states[0].turn_velocity = 0;
    for (int i = 1; i < (int)states.size(); i++)
    {
        states[i].drive_velocity = calculate_velocity(states[i], states[i-1], constraints.max_speed, constraints.max_acceleration);
    }
    
    states[states.size() - 1].drive_velocity = 0;
    states[states.size() - 1].turn_velocity = 0;
    for (int i = (int)states.size() - 2; i > 0; i--)
    {
        states[i].drive_velocity =
            std::min(states[i].drive_velocity, calculate_velocity(states[i], states[i+1], constraints.max_speed, constraints.max_acceleration));
    }

    states[0].time = 0;
    for (int i = 1; i < (int)states.size(); i++)
    {
        double velocity_initial = states[i-1].drive_velocity;
        double velocity_final = states[i].drive_velocity;
        double velocity_avg = (velocity_initial + velocity_final) / 2.0;

        double distance = states[i-1].position.distance_to(states[i].position);

        double delta_time = distance / velocity_avg;

        states[i].time = states[i-1].time + delta_time;
    }

    return Trajectory(states);
}