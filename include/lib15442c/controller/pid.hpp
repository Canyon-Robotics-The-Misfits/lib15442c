#pragma once

#include <cmath>

namespace lib15442c
{
    struct PIDParameters
    {
        double kP;
        double kI;
        double kD;

        double integral_active_zone = INFINITY;
        double integral_max = INFINITY;
        double integral_reset_zone = 0;
        bool reset_integral_on_cross = false;
    };

    class PID
    {
    private:
        double last_error = INFINITY;
        double total_error = 0;

    public:
        double kP;
        double kI;
        double kD;

        double integral_active_zone;
        double integral_max;
        double integral_reset_zone;
        bool reset_integral_on_cross;

        PID(PIDParameters parameters) :
            kP(parameters.kP), kI(parameters.kI), kD(parameters.kD),
            integral_active_zone(parameters.integral_active_zone),
            integral_max(parameters.integral_max),
            integral_reset_zone(parameters.integral_reset_zone),
            reset_integral_on_cross(parameters.reset_integral_on_cross) {};

        /**
         * @brief Calculate the output of the PID
         *
         * @param current The current value
         * @param target The target value
         * @return double The PID output
         */
        double calculate(double current, double target);

        /**
         * @brief Calculate the output of the PID
         *
         * @param error The error from the target
         * @return double The PID output
         */
        double calculateError(double error);

        /**
         * @brief Reset the PID total_error and last_error
         *
         */
        void reset();
    };
} // namespace lib15442c