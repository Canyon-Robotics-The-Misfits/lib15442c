#pragma once

#include "../device/motor.hpp"
#include "drivetrain.hpp"

#include <memory>

namespace lib15442c
{
    /**
     * A "Tank" Drivetrain
     */
    class TankDrive : public virtual IDrivetrain
    {
    private:
        // Drivetrain Motors
        std::shared_ptr<lib15442c::IMotor> left_motors;
        std::shared_ptr<lib15442c::IMotor> right_motors;

        // Settings
        double track_width;
        double deg_inch_ratio;
        FeedforwardConstants feedforward_constants;

    public:
        TankDrive(
            std::shared_ptr<lib15442c::IMotor> left_motors,
            std::shared_ptr<lib15442c::IMotor> right_motors,
            double wheel_diameter,
            double gear_ratio,
            double track_width,
            FeedforwardConstants feedforward_constants
        );
        
        /**
         * Move a skid-steer drive with a set left and right side voltage
         * @param left_speed The voltage of the left side
         * @param right_speed The voltage of the right side
         */
        void tank(double left_speed, double right_speed);
        /**
         * Move a drivetrain with a set linear and rotational voltage
         *
         * @param linear_speed The voltage to drive forward/back with
         * @param turn_speed The voltage to turn at
         */
        void move(double linear_speed, double turn_speed);
        /**
         * Move a drivetrain with a set linear and rotational voltage, prioritizing turning over driving in case the max voltage is exceded
         *
         * @param linear_speed The voltage to drive forward/back with
         * @param turn_speed The voltage to turn at
         */
        void move_ratio(double linear_speed, double turn_speed);
        /**
         * Move a drivetrain with a set linear and rotational speed (in/s)
         *
         * @param linear_velocity The speed to drive forward/back with
         * @param turn_velocity The speed to turn at
         * @param linear_accel The linear acceleration
         * @param turn_accel The rotational acceleration
         */
        void move_speed(double linear_velocity, double turn_velocity, double linear_accel = 0, double turn_accel = 0);

        /**
         * Set the brake mode of the drivetrain motors
         *
         * @param mode The mode to use
         */
        void set_brake_mode(lib15442c::MotorBrakeMode mode);
        /**
         * Get the brake mode of the drivetrain motors
         *
         * @return The mode the motors are on
         */
        lib15442c::MotorBrakeMode get_brake_mode();

        /**
         * @brief Get the highest motor tempature in the drivetrain
         * 
         * @return double The highest motor tempature
         */
        double max_temp();
        /**
         * Get the track width of the drivetrain
         *
         * @return The track width
         */
        double get_track_width();

        /**
         * @brief Check if the drivetrain motors are installed
         * 
         * @return bool
         */
        bool is_installed();

        /**
         * @brief List by port which motors aren't installed
         * 
         * @return std::vector<int> 
         */
        std::vector<int> get_uninstalled_motors();
    };
}