#pragma once

#include <cmath>

namespace lib15442c
{
    class PID {
    private:
        double last_error = INFINITY;
        double total_error = 0;

    public:
        double kP;
        double kI;
        double kD;

        PID(double kP, double kI, double kD): kP(kP), kI(kI), kD(kD) {};

        /**
         * @brief Calculate the output of the PID
         * 
         * @param current The current value
         * @param target The target value
         * @return double The PID output
         */
        double calculate(double current, double target);

        /**
         * @brief Calculate the outpit of the PID
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