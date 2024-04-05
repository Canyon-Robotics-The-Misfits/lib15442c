#pragma once

#include "vector.hpp"
#include "angle.hpp"

namespace lib15442c
{
    class Pose {
    public:
        double x;
        double y;
        Angle angle;

        Pose(double x, double y, Angle angle) : x(x), y(y), angle(angle) {};
        
        /**
         * @brief Return the angle in radians
         * 
         * @return double The angle
         */
        double rad();
        /**
         * @brief Return the angle in degrees
         * 
         * @return double The angle
         */
        double deg();

        /**
         * @brief Get the x and y as a Vec
         * 
         * @return the Vec
         */
        Vec vec();


        void operator=(const Pose& rhs);

        Pose operator+(const Vec& rhs);
        Pose operator-(const Vec& rhs);
        void operator+=(const Vec& rhs);
        void operator-=(const Vec& rhs);
        
        Pose operator+(const Pose& rhs);
        Pose operator-(const Pose& rhs);
        void operator+=(const Pose& rhs);
        void operator-=(const Pose& rhs);

        Pose operator*(const double& rhs);
        Pose operator/(const double& rhs);

        void operator*=(const double& rhs);
        void operator/=(const double& rhs);
    };

    #define pos(x, y) lib15442c::Pose(x, y, lib15442c::Angle::from_rad(INFINITY))
    #define posa(x, y, theta) lib15442c::Pose(x, y, theta)
}
