#pragma once

#include <variant>
#include <vector>

#include "lib15442c/math/vector.hpp"
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

    struct CircleZone
    {
        // The center of the circle
        Vec center;
        // The radius of the circle
        double radius;
        // The value to apply over the zone
        double value;
    };

    struct RectangleZone
    {
        // One corner of the rectangle
        Vec corner_a;
        // The other corner of the rectangle
        Vec corner_b;
        // The value to apply over the zone
        double value;
    };

    // A generic 2d zone with a value
    using Zone = std::variant<CircleZone, RectangleZone>;

    // Robot constraints and important info for trajectory computation
    struct TrajectoryConstraints
    {
        double max_speed;
        double max_acceleration;

        double track_width;
    };

    class TrajectoryBuilder {
    private:
        std::vector<HermitePair> hermite_spline;
        std::vector<Zone> max_speed_zones;

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
         * @param resolution How closely points should be to each other (inches)
         * @return Trajectory The computed trajectory
         */
        Trajectory compute(TrajectoryConstraints constraints, double resolution);
    };
} // namespace lib15442c
