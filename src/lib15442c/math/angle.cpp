#include "angle.hpp"
#include "math.hpp"

using lib15442c::Angle;

double Angle::wrap(double angle) {
    if (std::isnan(angle))
        return angle;

    return fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

Angle Angle::from_rad(double rad) {
    return Angle(wrap(rad));
}
Angle Angle::from_deg(double deg) {
    return Angle(wrap(deg * M_PI / 180.0));
}

double Angle::rad() {
    return wrap(theta);
}
double Angle::deg() {
    return wrap(theta) * 180.0 / M_PI;
}

Angle Angle::error_from(Angle target) {
    double a = deg();
    double b = target.deg();

    double error = a - b;
    
    if (fabs(error) > 180)
        error -= sgn(error) * 360;

    return Angle::from_deg(error);
}

Angle Angle::operator+(const Angle& rhs) {
    return Angle::from_rad(theta + rhs.theta);
}
Angle Angle::operator-(const Angle& rhs) {
    return Angle::from_rad(theta - rhs.theta);
}
Angle Angle::operator*(const Angle& rhs) {
    return Angle::from_rad(theta * rhs.theta);
}
Angle Angle::operator/(const Angle& rhs) {
    return Angle::from_rad(theta / rhs.theta);
}

void Angle::operator=(const Angle& rhs) {
    theta = rhs.theta;
}

void Angle::operator+=(const Angle& rhs) {
    theta = wrap(theta + rhs.theta);
}
void Angle::operator-=(const Angle& rhs) {
    theta = wrap(theta - rhs.theta);
}
void Angle::operator*=(const Angle& rhs) {
    theta = wrap(theta * rhs.theta);
}
void Angle::operator/=(const Angle& rhs) {
    theta = wrap(theta / rhs.theta);
}