#pragma once

#include <vector>

#include "lib15442c/math/vector.hpp"

namespace lib15442c
{
    struct TrajectoryState
    {
        // The position in a trajectory
        Vec position;
        // The target speed of the left side of the drive
        double left_speed;
        // The target speed of the right side of the drive
        double right_speed;
    };

    class Trajectory {
    friend class TrajectoryBuilder;
    protected: 
        std::vector<TrajectoryState> states;

        Trajectory(std::vector<TrajectoryState> states);

    public:
        /**
         * @brief Get the target state of the robot at a specified time
         * 
         * @param time The time
         * @return TrajectoryState The target state
         */
        TrajectoryState get_state(double time);

        /**
         * @brief Get how long the trajectory should take to follow
         * 
         * @return double How long it takes to follow the trajectory (ms)
         */
        double get_total_time();
    };
} // namespace lib15442c
