#include <iostream>
#include "pros/rtos.h"

#include "trajectory_builder.hpp"
#include "lib15442c/logger.hpp"
#include "lib15442c/math/math.hpp"

#define LOGGER "trajectory_builder.cpp"

lib15442c::Zone lib15442c::circle_zone(lib15442c::Vec center, double radius, double value)
{
    
    return [center, radius, value](lib15442c::Vec point)
    {
        if (point.distance_to_squared(center) < radius * radius)
        {
            return value;
        }
        else
        {
            return (double)INFINITY;
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
            return (double)INFINITY;
        }
    };
}



lib15442c::Vec lib15442c::TrajectoryBuilder::lerp_hermite(double t, Vec p0, Vec m0, Vec p1, Vec m1)
{
    return
        p0 * (2 * (t * t * t) - 3 * (t * t) + 1) +
        m0 * ((t * t * t) - 2 * (t * t) + t) +
        p1 * (-2 * (t * t * t) + 3 * (t * t)) +
        m1 * ((t * t * t) - (t * t));
}

std::vector<lib15442c::TrajectoryState> lib15442c::TrajectoryBuilder::calculate_hermite(int resolution, Vec p0, Vec m0, Vec p1, Vec m1)
{
    std::vector<TrajectoryState> states;

    if (resolution == -1)
    {
        // roughly auto-caluclate the resolution based on the size of the curve to put about one state per inch.
        // approximation is kinda bad for extreme curves which go far outside of the range of the start/end points,
        // but is suprisingly good within the range
        resolution = floor(std::max(std::abs((p0 - p1).x), std::abs((p0 - p1).y)) * 1.0);
    }

    for (int i = 1; i <= resolution; i++)
    {
        double dt = 1.0 / (double)resolution;
        double t = (double)i * dt;

        if (t > 1.0 - 0.0001)
        {
            t = 1.0;
        }

        Vec next = lerp_hermite(t, p0, m0, p1, m1);
        
        states.push_back({
            position: next,
            drive_velocity: INFINITY,
            rotational_velocity: INFINITY,
            drive_accel: INFINITY,
            rotational_accel: INFINITY,
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

        if (value < min_value)
        {
            min_value = value;
        }
    }

    return min_value;
}

double lib15442c::TrajectoryBuilder::calculate_velocity(TrajectoryState final, TrajectoryState initial, double physical_max_speed, double starting_acceleration)
{
    double velocity_initial = initial.drive_velocity;
    double distance = initial.position.distance_to(final.position);

    // accel should decrease as velocity approaches max speed
    double max_acceleration = starting_acceleration - (starting_acceleration / physical_max_speed) * velocity_initial;

    double delta_time = (-velocity_initial + sqrt(velocity_initial * velocity_initial + 2 * distance * max_acceleration)) / max_acceleration;

    double max_speed = std::min(physical_max_speed, get_max_speed(final.position));

    // std::cout << max_acceleration << ", " << velocity_initial + max_acceleration * delta_time << std::endl;

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

lib15442c::Trajectory lib15442c::TrajectoryBuilder::compute(TrajectoryConstraints constraints, int resolution, bool benchmark)
{
    std::vector<TrajectoryState> states;

    double start_time = pros::c::micros() / 1000.0;

    states.push_back({
        position: hermite_spline[0].point,
        drive_velocity: INFINITY,
        rotational_velocity: INFINITY,
        time: INFINITY
    });

    for (int i = 1; i < (int)hermite_spline.size(); i++)
    {

        std::vector<TrajectoryState> segment = 
            calculate_hermite(resolution, hermite_spline[i-1].point, hermite_spline[i-1].tangent, hermite_spline[i].point, hermite_spline[i].tangent);

        states.insert(states.end(), segment.begin(), segment.end());
    }

    double hermite_end_time = pros::c::micros() / 1000.0;

    states[0].drive_velocity = 0;
    states[0].rotational_velocity = 0;
    for (int i = 1; i < (int)states.size(); i++)
    {
        states[i].drive_velocity = calculate_velocity(states[i], states[i-1], constraints.max_speed, constraints.starting_acceleration);
    }

    double velocity_1_end_time = pros::c::micros() / 1000.0;
    
    states[states.size() - 1].drive_velocity = 0;
    states[states.size() - 1].rotational_velocity = 0;
    for (int i = (int)states.size() - 2; i > 0; i--)
    {
        states[i].drive_velocity =
            std::min(states[i].drive_velocity, calculate_velocity(states[i], states[i+1], constraints.max_speed, constraints.starting_acceleration));
    }

    double velocity_2_end_time = pros::c::micros() / 1000.0;

    states[0].time = 0;

    states[0].heading = Angle::from_rad(atan2(hermite_spline[0].tangent.x, hermite_spline[0].tangent.y)); // x and y swapped to make 0 degrees face in the direction of the y-axis
    states[states.size()-1].heading = Angle::from_rad(atan2(hermite_spline[hermite_spline.size()-1].tangent.x, hermite_spline[hermite_spline.size()-1].tangent.y)); // x and y swapped to make 0 degrees face in the direction of the y-axis
    for (int i = 1; i < (int)states.size(); i++)
    {
        double velocity_initial = states[i-1].drive_velocity;
        double velocity_final = states[i].drive_velocity;
        double velocity_avg = (velocity_initial + velocity_final) / 2.0;

        double distance = states[i-1].position.distance_to(states[i].position);

        double delta_time = distance / velocity_avg;

        states[i].time = states[i-1].time + delta_time;

        if (i != (int)states.size() - 1)
        {
            Vec prev_pos = states[i-1].position;
            Vec current_pos = states[i].position;
            Vec next_pos = states[i+1].position;

            Angle angle_to_next = Angle::from_rad(atan2(next_pos.x - current_pos.x, next_pos.y - current_pos.y)); // x and y swapped to make 0 degrees face in the direction of the y-axis

            states[i].heading = (states[i-1].heading + angle_to_next) / 2.0;

            // solve for menger curvature
            // a = states[i-1].position = prev_pos
            // b = states[i].position = current_pos
            // c = states[i+1].position = next_pos
            double triangle_area_doubled = 
                (current_pos.x - prev_pos.x) * (next_pos.y - prev_pos.y) -
                (current_pos.y - prev_pos.y) * (next_pos.x - prev_pos.x);
            double dist_a_b = prev_pos.distance_to_squared(current_pos);
            double dist_b_c = current_pos.distance_to_squared(next_pos);
            double dist_c_a = next_pos.distance_to_squared(prev_pos);

            double curvature = sgn(triangle_area_doubled) * sqrt((triangle_area_doubled * triangle_area_doubled) / (dist_a_b * dist_b_c * dist_c_a));

            double rotational_velocity = states[i].drive_velocity * curvature;

            if (std::abs(states[i].drive_velocity - constraints.track_width * rotational_velocity) > constraints.max_speed)
            {
                states[i].drive_velocity = lib15442c::sgn(states[i].drive_velocity) * constraints.max_speed / (1 - curvature * constraints.track_width);
                rotational_velocity = states[i].drive_velocity * curvature;
            }
            else if (std::abs(states[i].drive_velocity + constraints.track_width * rotational_velocity) > constraints.max_speed)
            {
                states[i].drive_velocity = lib15442c::sgn(states[i].drive_velocity) * constraints.max_speed / (1 + curvature * constraints.track_width);
                rotational_velocity = states[i].drive_velocity * curvature;
            }

            states[i].rotational_velocity = rotational_velocity;
        }
    }

    double rotation_pass_end_time = pros::c::micros() / 1000.0;

    states[0].drive_accel = states[1].drive_velocity / states[1].time;
    states[0].rotational_accel = states[1].rotational_velocity / states[1].time;
    states[states.size()-1].drive_accel = -states[states.size()-2].drive_velocity / (states[states.size()-1].time - states[states.size()-2].time);
    states[states.size()-1].rotational_accel = -states[states.size()-2].rotational_velocity / (states[states.size()-1].time - states[states.size()-2].time);
    for (int i = 1; i < (int)states.size()-1; i++)
    {
        states[i].drive_accel = (states[i+1].drive_velocity - states[i-1].drive_velocity) / (states[i+1].time - states[i-1].time);
        states[i].rotational_accel = (states[i+1].rotational_velocity - states[i-1].rotational_velocity) / (states[i+1].time - states[i-1].time);
    }

    double end_time = pros::c::micros() / 1000.0;

    if (benchmark)
    {
        DEBUG_TEXT("---- TRAJECTORY BENCHMARK ----");
        DEBUG("TOTAL: %f", end_time - start_time);
        DEBUG("hermite calculation: %f", hermite_end_time - start_time);
        DEBUG("velocity pass: %f", velocity_2_end_time - hermite_end_time);
        DEBUG("    forward: %f", velocity_1_end_time - hermite_end_time);
        DEBUG("    reverse: %f", velocity_2_end_time - velocity_1_end_time);
        DEBUG("rotation pass: %f", rotation_pass_end_time - velocity_2_end_time);
        DEBUG("accel pass: %f", end_time - rotation_pass_end_time);
        DEBUG_TEXT("------------------------------");
    }

    return Trajectory(states);
}