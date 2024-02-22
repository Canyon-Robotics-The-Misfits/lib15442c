#include "lib15442c/chasis/tank_drive.hpp"
#include <vector>
#include <cmath>

lib15442c::TankDrive::TankDrive(
    std::shared_ptr<pros::v5::AbstractMotor> left_motors,
    std::shared_ptr<pros::v5::AbstractMotor> right_motors,
    double wheel_diameter,
    double gear_ratio,
    double track_width) : left_motors(left_motors),
                         right_motors(right_motors),
                         track_width(track_width),
                         deg_inch_ratio(wheel_diameter * M_PI * gear_ratio / 360.0)
{
    left_motors->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    left_motors->tare_position();
    right_motors->tare_position();
}


void lib15442c::TankDrive::tank(double left_speed, double right_speed) {
    left_motors->move(left_speed);
    right_motors->move(right_speed);
}

void lib15442c::TankDrive::move(double linear_speed, double turn_speed) {
    tank(
        linear_speed + turn_speed,
        linear_speed - turn_speed
    );
}

void lib15442c::TankDrive::move_speed(double linear_speed, double turn_speed) {
    // TODO
}

void lib15442c::TankDrive::set_brake_mode(pros::v5::MotorBrake mode) {
    left_motors->set_brake_mode(mode);
    right_motors->set_brake_mode(mode);
}

pros::v5::MotorBrake lib15442c::TankDrive::get_brake_mode() {
    return left_motors->get_brake_mode();
}

std::vector<double> lib15442c::TankDrive::get_temps() {
    auto left = left_motors->get_temperature_all();
    auto right = right_motors->get_temperature_all();

    left.insert( left.end(), right.begin(), right.end() );

    return left;
}