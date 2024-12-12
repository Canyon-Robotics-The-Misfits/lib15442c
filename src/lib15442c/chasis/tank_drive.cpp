#include "lib15442c/chasis/tank_drive.hpp"
#include <vector>
#include <cmath>

lib15442c::TankDrive::TankDrive(
    std::shared_ptr<lib15442c::IMotor> left_motors,
    std::shared_ptr<lib15442c::IMotor> right_motors,
    double wheel_diameter,
    double gear_ratio,
    double track_width) : left_motors(left_motors),
                         right_motors(right_motors),
                         track_width(track_width),
                         deg_inch_ratio(wheel_diameter * M_PI * gear_ratio / 360.0)
{
    left_motors->set_brake_mode(MotorBrakeMode::COAST);
    right_motors->set_brake_mode(MotorBrakeMode::COAST);

    // left_motors->tare_position();
    // right_motors->tare_position();
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
void lib15442c::TankDrive::move_ratio(double linear_speed, double turn_speed) {
    if (fabs(linear_speed + turn_speed) > 127)
    {
        float total_speed = fabs(linear_speed + turn_speed);
        linear_speed = linear_speed / total_speed * 127;
        turn_speed = turn_speed / total_speed * 127;
    }
    else if (fabs(linear_speed - turn_speed) > 127)
    {
        float total_speed = fabs(linear_speed - turn_speed);
        linear_speed = linear_speed / total_speed * 127;
        turn_speed = turn_speed / total_speed * 127;
    }

    tank(
        linear_speed + turn_speed,
        linear_speed - turn_speed
    );
}

void lib15442c::TankDrive::move_speed(double linear_speed, double turn_speed) {
    // TODO
}

void lib15442c::TankDrive::set_brake_mode(MotorBrakeMode mode) {
    left_motors->set_brake_mode(mode);
    right_motors->set_brake_mode(mode);
}

lib15442c::MotorBrakeMode lib15442c::TankDrive::get_brake_mode() {
    return left_motors->get_brake_mode();
}

double lib15442c::TankDrive::max_temp() {
    auto left = left_motors->get_temp();
    auto right = right_motors->get_temp();

    return std::max(left, right);
}

double lib15442c::TankDrive::get_track_width() {
    return track_width;
}

bool lib15442c::TankDrive::is_installed()
{
    return left_motors->is_installed() && right_motors->is_installed();
}

std::vector<int> lib15442c::TankDrive::get_uninstalled_motors()
{
    auto left = left_motors->get_uninstalled_motors();
    auto right = right_motors->get_uninstalled_motors();

    left.insert( left.end(), right.begin(), right.end() );

    return left;
}