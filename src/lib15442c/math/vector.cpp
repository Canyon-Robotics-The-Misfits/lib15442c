#include "vector.hpp"
#include "math.hpp"

using lib15442c::Vec;

double Vec::magnitude() {
    return sqrt(x * x + y * y);
}
lib15442c::Angle Vec::angle() {
    return Angle::from_rad(atan2(y, x));
}

lib15442c::Angle Vec::angle_to(Vec vector) {
    return Angle::from_rad(atan2(vector.y - y, vector.x - x));
}

double Vec::distance_to(Vec vector) {
    return sqrt(pow(vector.x - x, 2) + pow(vector.y - y, 2));
}

Vec Vec::normalized() {
    Angle vec_angle = angle();

    return Vec(cos(vec_angle.rad()), sin(vec_angle.rad()));
}

// Vec + Vec operators
Vec Vec::operator+(const Vec& rhs)
{
    return Vec(x + rhs.x, y + rhs.y);
}
Vec Vec::operator-(const Vec& rhs)
{
    return Vec(x - rhs.x, y - rhs.y);
}

// Vec + Vec assignment operators
void Vec::operator+=(const Vec& rhs)
{
    x += rhs.x;
    y += rhs.y;
}
void Vec::operator-=(const Vec& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
}

// Vec + float operators
Vec Vec::operator*(const double& rhs)
{
    return Vec(x * rhs, y * rhs);
}
Vec Vec::operator/(const double& rhs)
{
    return Vec(x / rhs, y / rhs);
}
void Vec::operator*=(const double& rhs)
{
    x *= rhs;
    y *= rhs;
}
void Vec::operator/=(const double& rhs)
{
    x /= rhs;
    y /= rhs;
}