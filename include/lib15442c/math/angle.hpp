#pragma once

#include <cmath>

namespace lib15442c {
    class Angle {
    protected:
        // The angle, in degrees from -180 to 180
        double theta;

        static double wrap(double angle);

        Angle(double theta): theta(theta) {};

    public:
        Angle(): theta(INFINITY) {};

        /**
         * @brief Create a non-existant angle
         * 
         * @return Angle The angle
         */
        static Angle none();
        /**
         * @brief Create an Angle from a value in radians
         * 
         * @param rad The value in radians
         * @return Angle The angle 
         */
        static Angle from_rad(double rad);
        /**
         * @brief Create an Angle from a value in degrees
         * 
         * @param deg The value in degrees
         * @return Angle The angle
         */
        static Angle from_deg(double deg);

        /**
         * @brief Return the value of the angle in radians
         * 
         * @return double The angle
         */
        double rad();
        /**
         * @brief Return the value of the angle in degrees
         * 
         * @return double The angle
         */
        double deg();

        /**
         * @brief Whether the angle is non-existant
         * 
         * @return true The angle doesn't have a value
         * @return false The angle has a value
         */
        bool is_none();

        /**
         * @brief Get the error from the current angle to a target angle
         * 
         * @param target The target angle
         * @return double The error
         */
        lib15442c::Angle error_from(lib15442c::Angle target);

        void operator=(const lib15442c::Angle& rhs);
        
        lib15442c::Angle operator-();

        lib15442c::Angle operator+(const lib15442c::Angle& rhs);
        lib15442c::Angle operator-(const lib15442c::Angle& rhs);
        lib15442c::Angle operator*(const lib15442c::Angle& rhs);
        lib15442c::Angle operator/(const lib15442c::Angle& rhs);
        
        lib15442c::Angle operator*(const double& rhs);
        lib15442c::Angle operator/(const double& rhs);

        void operator+=(const lib15442c::Angle& rhs);
        void operator-=(const lib15442c::Angle& rhs);
        void operator*=(const lib15442c::Angle& rhs);
        void operator/=(const lib15442c::Angle& rhs);
        
        void operator*=(const double& rhs);
        void operator/=(const double& rhs);

        bool operator==(const lib15442c::Angle& rhs);
        bool operator<(const lib15442c::Angle& rhs);
        bool operator>(const lib15442c::Angle& rhs);
        bool operator<=(const lib15442c::Angle& rhs);
        bool operator>=(const lib15442c::Angle& rhs);
    };

    inline namespace literals {
        /**
         * @brief Create an Angle from a radian value
         * 
         * @param double The radian value
         * @return Angle the angle
         */
        lib15442c::Angle operator ""_rad(long double);
        lib15442c::Angle operator ""_rad(unsigned long long);
        /**
         * @brief Create an Angle from a degree value
         * 
         * @param double The degree value
         * @return Angle the angle
         */
        lib15442c::Angle operator ""_deg(long double);
        lib15442c::Angle operator ""_deg(unsigned long long);
    }
}