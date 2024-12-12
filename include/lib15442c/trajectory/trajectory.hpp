#pragma once

#include <vector>

#include "lib15442c/math/vector.hpp"

namespace lib15442c
{
    struct TrajectoryState
    {
        // The position in a trajectory
        Vec position;
        // The target drive velocity (in/s)
        double drive_velocity;
        // The target turn velocity (rad/s)
        double rotational_velocity;
        // The time the state is at
        double time;
        // The angle the robot should be facing
        Angle heading;
    };

    class Trajectory {
    friend class TrajectoryBuilder;
    private:
        std::vector<TrajectoryState> states;

        static TrajectoryState lerp_state(TrajectoryState a, TrajectoryState b, double t);
    protected: 
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

        void debug_log();
    };
} // namespace lib15442c
