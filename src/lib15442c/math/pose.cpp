#include "pose.hpp"
#include "vector.hpp"

using lib15442c::Pose;
using lib15442c::Vec;
using lib15442c::Angle;

Pose Pose::none() {
    return Pose(
        INFINITY,
        INFINITY,
        Angle::none()
    );
}

bool Pose::is_none() {
    return (x == INFINITY) && (y == INFINITY) && angle.is_none();
}

double Pose::rad() {
    return angle.rad();
}
double Pose::deg() {
    return angle.deg();
}

Vec Pose::vec() {
    return Vec(x, y);
}

void Pose::operator=(const Pose& rhs) {
    x = rhs.x;
    y = rhs.y;
    angle = rhs.angle;
}


Pose Pose::operator+(const Vec& rhs)
{
    return Pose(x + rhs.x, y + rhs.y, angle);
}
Pose Pose::operator-(const Vec& rhs)
{
    return Pose(x - rhs.x, y - rhs.y, angle);
}

void Pose::operator+=(const Vec& rhs)
{
    x += rhs.x;
    y += rhs.y;
}
void Pose::operator-=(const Vec& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
}


Pose Pose::operator+(const Pose& rhs)
{
    return Pose(x + rhs.x, y + rhs.y, angle);
}
Pose Pose::operator-(const Pose& rhs)
{
    return Pose(x - rhs.x, y - rhs.y, angle);
}

void Pose::operator+=(const Pose& rhs)
{
    x += rhs.x;
    y += rhs.y;
}
void Pose::operator-=(const Pose& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
}


Pose Pose::operator*(const double& rhs)
{
    return Pose(x * rhs, y * rhs, angle);
}
Pose Pose::operator/(const double& rhs)
{
    return Pose(x / rhs, y / rhs, angle);
}
void Pose::operator*=(const double& rhs)
{
    x *= rhs;
    y *= rhs;
}
void Pose::operator/=(const double& rhs)
{
    x /= rhs;
    y /= rhs;
}