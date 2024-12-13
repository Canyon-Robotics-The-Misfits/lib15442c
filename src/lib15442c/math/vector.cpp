#include "vector.hpp"
#include "math.hpp"

using lib15442c::Vec;

double Vec::magnitude() {
    return sqrt(x * x + y * y);
}
lib15442c::Angle Vec::angle() {
    return Angle::from_rad(atan2(x, y)); // x and y swapped to make 0 degrees face in the direction of the y-axis
}

lib15442c::Angle Vec::angle_to(Vec vector) {
    return Angle::from_rad(atan2(vector.x - x, vector.y - y)); // x and y swapped to make 0 degrees face in the direction of the y-axis
}

double Vec::distance_to(Vec vector) {
    return sqrt(distance_to_squared(vector));
}
double Vec::distance_to_squared(Vec vector) {
    return (vector.x - x) * (vector.x - x) + (vector.y - y, 2) * (vector.y - y, 2);
}

Vec Vec::normalized() {
    return *this / magnitude();
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

double Vec::dot(Vec& rhs)
{
    return (x * rhs.x) + (y * rhs.y);
}