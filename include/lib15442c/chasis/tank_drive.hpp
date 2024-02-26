#pragma once

#include "pros/abstract_motor.hpp"
#include "drivetrain.hpp"

namespace lib15442c
{
    class TankDrive : virtual public IDrivetrain
    {
    private:
        // Drivetrain Motors
        std::shared_ptr<pros::v5::AbstractMotor> left_motors;
        std::shared_ptr<pros::v5::AbstractMotor> right_motors;

        // Settings
        double track_width;
        double deg_inch_ratio;

    public:
        TankDrive(
            std::shared_ptr<pros::AbstractMotor> left_motors,
            std::shared_ptr<pros::AbstractMotor> right_motors,
            double wheel_diameter,
            double gear_ratio,
            double track_width
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
         * Move a drivetrain with a set linear and rotational speed (in/s)
         *
         * @param linear_speed The speed to drive forward/back with
         * @param turn_speed The speed to turn at
         */
        void move_speed(double linear_speed, double turn_speed);

        /**
         * Set the brake mode of the drivetrain motors
         *
         * @param mode The mode to use
         */
        void set_brake_mode(pros::v5::MotorBrake mode);
        /**
         * Get the brake mode of the drivetrain motors
         *
         * @return The mode the motors are on
         */
        pros::v5::MotorBrake get_brake_mode();

        /**
         * Get the tempatures of the motors
         *
         * @return A list of the tempatures
         */
        std::vector<double> get_temps();
        /**
         * Get the track width of the drivetrain
         *
         * @return The track width
         */
        double get_track_width();
    };
}