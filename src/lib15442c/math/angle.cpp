#include "angle.hpp"
#include "math.hpp"

using lib15442c::Angle;

double Angle::wrap(double angle) {
    if (std::isnan(angle) || angle == INFINITY)
        return angle;

    return fmod(angle + 180, 360) - 180;
}

Angle Angle::none() {
    return Angle(INFINITY);
}
Angle Angle::from_rad(double rad) {
    return Angle(wrap(rad * 180.0 / M_PI));
}
Angle Angle::from_deg(double deg) {
    return Angle(wrap(deg));
}

bool Angle::is_none() {
    return theta == INFINITY || std::isnan(theta);
}

double Angle::rad() {
    return wrap(theta) * M_PI / 180.0;
}
double Angle::deg() {
    return wrap(theta);
}

Angle Angle::error_from(Angle target) {
    double a = deg();
    double b = target.deg();

    double error = b - a;
    
    if (fabs(error) > 180)
        error -= sgn(error) * 360;

    return Angle::from_deg(error);
}

Angle Angle::operator+(const Angle& rhs) {
    return Angle::from_deg(theta + rhs.theta);
}
Angle Angle::operator-(const Angle& rhs) {
    return Angle::from_deg(theta - rhs.theta);
}
Angle Angle::operator*(const Angle& rhs) {
    return Angle::from_deg(theta * rhs.theta);
}
Angle Angle::operator/(const Angle& rhs) {
    return Angle::from_deg(theta / rhs.theta);
}

Angle Angle::operator*(const double& rhs) {
    return Angle::from_deg(theta * rhs);
}
Angle Angle::operator/(const double& rhs) {
    return Angle::from_deg(theta / rhs);
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

void Angle::operator*=(const double& rhs) {
    theta = wrap(theta * rhs);
}
void Angle::operator/=(const double& rhs) {
    theta = wrap(theta / rhs);
}


bool Angle::operator==(const Angle& rhs) {
    return theta == rhs.theta;
}
bool Angle::operator>(const Angle& rhs) {
    return this->operator-(rhs).theta > 0;
}
bool Angle::operator<(const Angle& rhs) {
    return this->operator-(rhs).theta < 0;
}
bool Angle::operator>=(const Angle& rhs) {
    return this->operator-(rhs).theta >= 0;
}
bool Angle::operator<=(const Angle& rhs) {
    return this->operator-(rhs).theta <= 0;
}


Angle lib15442c::literals::operator ""_rad(long double value) {
    return Angle::from_rad(value);
}
Angle lib15442c::literals::operator ""_rad(unsigned long long value) {
    return Angle::from_rad(value);
}
Angle lib15442c::literals::operator ""_deg(long double value) {
    return Angle::from_deg(value);
}
Angle lib15442c::literals::operator ""_deg(unsigned long long value) {
    return Angle::from_deg(value);
}