#pragma once

#include <cmath>

namespace lib15442c
{
    struct PIDParameters
    {
        /**
         * @brief Proportional constant
         */
        double kP;
        /**
         * @brief Integral constant
         */
        double kI;
        /**
         * @brief Derivative constant
         */
        double kD;

        /**
         * @brief The threshold the error must be within to start accumulating integral
         */
        double integral_active_zone = INFINITY;
        /**
         * @brief The maximum value the integral can be
         */
        double integral_max = INFINITY;
        /**
         * @brief The threshold in which the integral will get reset
         */
        double integral_reset_zone = 0;
        /**
         * @brief Whether to reset the integral if the error passes zero
         */
        bool reset_integral_on_cross = false;

        /**
         * @brief The maximum amount the output can change by per tick
         */
        double slew_rate = INFINITY;
    };

    /**
     * A generic PID controller
     */
    class PID
    {
    private:
        double last_error = INFINITY;
        double total_error = 0;

        double last_output = 0;

    public:
        double kP;
        double kI;
        double kD;

        double integral_active_zone;
        double integral_max;
        double integral_reset_zone;
        bool reset_integral_on_cross;

        double slew_rate;

        PID(PIDParameters parameters) :
            kP(parameters.kP), kI(parameters.kI), kD(parameters.kD),
            integral_active_zone(parameters.integral_active_zone),
            integral_max(parameters.integral_max),
            integral_reset_zone(parameters.integral_reset_zone),
            reset_integral_on_cross(parameters.reset_integral_on_cross),
            slew_rate(parameters.slew_rate) {};

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
        double calculate_error(double error);

        /**
         * @brief Reset the PID total_error and last_error
         */
        void reset_pid();
    };
} // namespace lib15442c