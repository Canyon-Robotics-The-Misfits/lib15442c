#pragma once

#include "../device/motor.hpp"
#include <vector>

namespace lib15442c
{
    struct FeedforwardConstants
    {
        // voltage required to overcome static friction
        double kS;
        // how much voltage to apply per in/s while maintaining speed
        double kV;
        // how much voltage to apply per in/s/s of acceleration
        double kA;
        double kA_down;
        // how much voltage to apply per in/s of error in velocity
        double kP;
    };

    /**
     * An abstract drivetrain
     */
    class IDrivetrain
    {
    public:
        /**
         * Move a skid-steer drive with a set left and right side voltage
         * @param left_speed The voltage of the left side
         * @param right_speed The voltage of the right side
         */
        virtual void tank(double left_speed, double right_speed) = 0;
        /**
         * Move a drivetrain with a set linear and rotational voltage
         *
         * @param linear_speed The voltage to drive forward/back with
         * @param turn_speed The voltage to turn with
         */
        virtual void move(double linear_speed, double turn_speed) = 0;
        /**
         * Move a drivetrain with a set linear and rotational voltage, prioritizing turning over driving in case the max voltage is exceded
         *
         * @param linear_speed The voltage to drive forward/back with
         * @param turn_speed The voltage to turn with
         */
        virtual void move_ratio(double linear_speed, double turn_speed) = 0;
        /**
         * Move a drivetrain with a set linear and rotational speed (in/s)
         *
         * @param linear_speed The speed to drive forward/back with
         * @param turn_speed The speed to turn with
         */
        virtual void move_speed(double linear_speed, double turn_speed, double linear_accel = 0, double turn_accel = 0) = 0;

        /**
         * Set the brake mode of the drivetrain motors
         *
         * @param mode The mode to use
         */
        virtual void set_brake_mode(lib15442c::MotorBrakeMode mode) = 0;
        /**
         * Get the brake mode of the drivetrain motors
         *
         * @return The mode the motors are on
         */
        virtual lib15442c::MotorBrakeMode get_brake_mode() = 0;

        /**
         * @brief Get the highest motor tempature in the drivetrain
         * 
         * @return double The highest motor tempature
         */
        virtual double max_temp() = 0;
        /**
         * Get the track width of the drivetrain
         *
         * @return The track width
         */
        virtual double get_track_width() = 0;
        
        /**
         * @brief Check if the drivetrain motors are installed
         * 
         * @return bool
         */
        virtual bool is_installed() = 0;
    };

}