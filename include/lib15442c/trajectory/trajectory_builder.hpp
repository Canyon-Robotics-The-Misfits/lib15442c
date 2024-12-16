#pragma once

#include <functional>
#include <vector>

#include "lib15442c/math/vector.hpp"
#include "lib15442c/chasis/drivetrain.hpp"
#include "trajectory.hpp"

namespace lib15442c
{
    struct HermitePair
    {
        // The point for the robot to drive through
        Vec point;
        // The tangent vector of the spline at the point
        Vec tangent;
    };

    // A function which returns a value based on a position. Should return INFINITY if it should have no effect
    using Zone = std::function<double (lib15442c::Vec)>;

    Zone circle_zone(Vec center, double radius, double value);
    Zone rect_zone(Vec corner_a, Vec corner_b, double value);

    class TrajectoryBuilder {
    private:
        std::vector<HermitePair> hermite_spline;
        std::vector<Zone> max_speed_zones;

        static Vec lerp_hermite(double t, Vec p0, Vec t0, Vec p1, Vec t1);

        static std::vector<TrajectoryState> calculate_hermite(int resolution, Vec p0, Vec t0, Vec p1, Vec t1);

        double get_max_speed(Vec position);

        double calculate_velocity(TrajectoryState final, TrajectoryState initial, double max_speed, double max_acceleration);

    public:
        /**
         * @brief Create a new trajectory builder. Uses cubic hermite splines
         * 
         * @param initial_point The initial position and tangent vectors of the spline
         */
        TrajectoryBuilder(HermitePair initial_point);

        /**
         * @brief Add a pair of position and tangent vectors to the hermite spline
         * 
         * @param pair The pair
         */
        void append_hermite(HermitePair pair);

        /**
         * @brief Add a zone where the max speed should be lower
         * 
         * @param zone The zone to reduce the max speed in
         */
        void add_max_speed_zone(Zone zone);

        /**
         * @brief Process the spline and return a computed trajectory motion profile
         * 
         * @param constraints Motion constraints for the robot
         * @param resolution How many points to calculate per curve. -1 to auto-calculate a resolution
         * @param benchmark Whether to log how long each step of the computation took
         * @return Trajectory The computed trajectory
         */
        Trajectory compute(DrivetrainConstraints constraints, int resolution = -1, bool benchmark = false);
    };
} // namespace lib15442c
