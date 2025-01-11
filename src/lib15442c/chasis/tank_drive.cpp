#include "lib15442c/chasis/tank_drive.hpp"
#include "lib15442c/math/math.hpp"
#include <vector>
#include <cmath>
#include <iostream>

lib15442c::TankDrive::TankDrive(
    std::shared_ptr<lib15442c::IMotor> left_motors,
    std::shared_ptr<lib15442c::IMotor> right_motors,
    double wheel_diameter,
    double gear_ratio,
    double track_width,
    FeedforwardConstants feedforward_constants) : left_motors(left_motors),
                         right_motors(right_motors),
                         track_width(track_width),
                         deg_inch_ratio(wheel_diameter * M_PI * gear_ratio / 360.0),
                         feedforward_constants(feedforward_constants)
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

void lib15442c::TankDrive::move_speed(double linear_velocity, double turn_velocity, double linear_accel, double turn_accel) {
    double target_left_velocity = linear_velocity - track_width * turn_velocity;
    double target_right_velocity = linear_velocity + track_width * turn_velocity;

    double target_left_accel = linear_accel - track_width * turn_accel;
    double target_right_accel = linear_accel + track_width * turn_accel;

    double current_left_velocity = left_motors->get_velocity() * deg_inch_ratio;
    double current_right_velocity = right_motors->get_velocity() * deg_inch_ratio;

    double left_error = target_left_velocity - current_left_velocity;
    double right_error = target_right_velocity - current_right_velocity;

    double left_pwm =
        feedforward_constants.kS * lib15442c::sgn(target_left_velocity) +
        target_left_velocity * feedforward_constants.kV +
        target_left_accel * (target_left_accel > 0 ? feedforward_constants.kA : feedforward_constants.kA_down) +
        left_error * feedforward_constants.kP;
    double right_pwm = 
        feedforward_constants.kS * lib15442c::sgn(target_right_velocity) +
        target_right_velocity * feedforward_constants.kV +
        target_right_accel * (target_right_accel > 0 ? feedforward_constants.kA : feedforward_constants.kA_down) +
        right_error * feedforward_constants.kP;

    // std::cout << target_left_velocity << ", " << target_left_accel << ", " << current_left_velocity << ", " << left_pwm << std::endl;

    left_motors->move(left_pwm);
    right_motors->move(right_pwm);
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