#include "pid.hpp"
#include "lib15442c/math/math.hpp"

double lib15442c::PID::calculate(double current, double target)
{
    return calculate_error(target - current);
}

double lib15442c::PID::calculate_error(double error, bool disable_i) {
    if (last_error == INFINITY) {
        last_error = error;
    }

    if (reset_integral_on_cross && lib15442c::sgn(error) != lib15442c::sgn(last_error)) {
        total_error = 0;
    }

    if (std::abs(error) < integral_active_zone) {
        total_error += error;
    } else {
        total_error = 0;
    }

    if (std::abs(total_error) > integral_max) {
        total_error = integral_max * sgn(total_error);
    }

    if (std::abs(error) < integral_reset_zone) {
        total_error = 0;
    }

    double proportional = error * kP;
    double integral = total_error * kI;
    double derivative = (error - last_error) * kD;

    last_error = error;

    double output = proportional + ( !disable_i ? integral : 0 ) + derivative;

    if (std::fabs(output - last_output) > slew_rate) {
        output = last_output + slew_rate * lib15442c::sgn(output - last_output);
    }

    last_output = output;

    return output;
}

void lib15442c::PID::reset_pid() {
    total_error = 0;
    last_error = INFINITY;
}