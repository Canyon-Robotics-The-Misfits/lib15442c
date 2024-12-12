#pragma once

#include "angle.hpp"

namespace lib15442c {
    /**
     * @brief A vector in 2D space
     */
    class Vec {
    protected:

        Vec(): x(0), y(0) {};

    public:
        double x;
        double y;


        Vec(double x, double y): x(x), y(y) {};

        /**
         * @brief Get the magnitude of the vector
         * 
         * @return double The magnitude
         */
        double magnitude();
        /**
         * @brief The angle of the vector
         * 
         * @return Angle The angle
         */
        lib15442c::Angle angle();

        /**
         * @brief The angle between the point defined by this vector and another one
         * 
         * @param vector The other one
         * @return Angle The angle
         */
        lib15442c::Angle angle_to(lib15442c::Vec vector);

        /**
         * @brief The distance between this vector and another one
         * 
         * @param vector The other one
         * @return double The distance
         */
        double distance_to(lib15442c::Vec vector);
        /**
         * @brief The distance between this vector and another one, squared to save on processing time
         * 
         * @param vector The other one
         * @return double The distance
         */
        double distance_to_squared(lib15442c::Vec vector);

        /**
         * @brief Returns the normalized vector (same angle, magnitude of 1)
         * 
         * @return Vec The normalized vecotr
         */
        lib15442c::Vec normalized();


        lib15442c::Vec operator+(const lib15442c::Vec& rhs);
        lib15442c::Vec operator-(const lib15442c::Vec& rhs);

        void operator+=(const lib15442c::Vec& rhs);
        void operator-=(const lib15442c::Vec& rhs);

        lib15442c::Vec operator*(const double& rhs);
        lib15442c::Vec operator/(const double& rhs);

        void operator*=(const double& rhs);
        void operator/=(const double& rhs);

        /**
         * @brief Calculate the dot product with another vector
         * 
         * @param rhs The other vector
         * @return double The dot product
         */
        double dot(Vec& rhs);
    };
}