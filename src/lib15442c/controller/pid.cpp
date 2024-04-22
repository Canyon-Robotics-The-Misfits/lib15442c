#include "pid.hpp"
#include "lib15442c/math/math.hpp"

double lib15442c::PID::calculate(double current, double target)
{
    return calculateError(target - current);
}

double lib15442c::PID::calculateError(double error) {
    if (last_error == INFINITY) {
        last_error = error;
    }

    double proportional = error * kP;
    double integral = total_error * kI;
    double derivative = (error - last_error) * kD;

    if (reset_integral_on_cross && lib15442c::sgn(error) != lib15442c::sgn(last_error)) {
        total_error = 0;
    }

    if (error < integral_active_zone) {
        total_error += error;
    } else {
        total_error = 0;
    }

    if (total_error > integral_max) {
        total_error = integral_max;
    }

    if (error < integral_reset_zone) {
        total_error = 0;
    }

    last_error = error;

    return proportional + integral + derivative;
}

void lib15442c::PID::reset() {
    total_error = 0;
    last_error = INFINITY;
}