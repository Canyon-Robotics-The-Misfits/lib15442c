#include "motor.hpp"
#include <iostream>

using lib15442c::MotorBrakeMode;

#ifndef LIB15442C_MOCK_DEVICES_ONLY

#include "pros/motors.h"

using lib15442c::Motor;
using lib15442c::MotorGroup;

Motor::Motor(MotorParameters parameters)
    : port(parameters.port), reversed(parameters.reversed), brake_mode(parameters.brake_mode), ratio(parameters.ratio)
{
    pros::c::motor_set_gearing(port, pros::E_MOTOR_GEAR_RED); // always set to red, modified to proper ratio in other funcs
    if (port < 0)
    {
        port = abs(port);
        reversed = !reversed;
    }
    set_brake_mode(brake_mode);
}

void Motor::move(double voltage)
{
    if (voltage == 0)
    {
        pros::c::motor_brake(port);
    }
    else
    {
        pros::c::motor_move(port, voltage * (reversed ? -1 : 1));
    }
}

void Motor::move_velocity(double velocity)
{
    pros::c::motor_move_velocity(port, velocity / ratio * MOTOR_RED * (reversed ? -1 : 1)); // multiply by MOTOR_RED because motors are configured as red ones
}

double Motor::get_velocity()
{
    return pros::c::motor_get_actual_velocity(port) / MOTOR_RED * ratio * 360.0 / 60.0 * (reversed ? -1 : 1);
}

void Motor::set_brake_mode(MotorBrakeMode brake_mode)
{
    this->brake_mode = brake_mode;

    switch (brake_mode)
    {
    case MotorBrakeMode::COAST:
        pros::c::motor_set_brake_mode(port, pros::E_MOTOR_BRAKE_COAST);
        break;

    case MotorBrakeMode::BRAKE:
        pros::c::motor_set_brake_mode(port, pros::E_MOTOR_BRAKE_BRAKE);
        break;

    case MotorBrakeMode::HOLD:
        pros::c::motor_set_brake_mode(port, pros::E_MOTOR_BRAKE_HOLD);
        break;
    default:
        break;
    }
}

MotorBrakeMode Motor::get_brake_mode()
{
    return brake_mode;
}

void Motor::set_reversed(bool reversed)
{
    this->reversed = reversed;
}

bool Motor::get_reversed()
{
    return reversed;
}

double Motor::get_temp()
{
    return pros::c::motor_get_temperature(port);
}

void Motor::set_ratio(double ratio)
{
    this->ratio = ratio;
}

double Motor::get_ratio()
{
    return ratio;
}

bool Motor::is_installed()
{
    return pros::c::motor_get_gearing(port) != pros::E_MOTOR_GEARSET_INVALID;
}

int Motor::get_port()
{
    return port;
}

std::vector<int> Motor::get_uninstalled_motors()
{
    return is_installed() ? std::vector<int>() : std::vector<int>({get_port()});
}


MotorGroup::MotorGroup(std::initializer_list<MotorParameters> parameters)
{
    for (auto parameter : parameters) {
        motors.push_back(std::make_unique<Motor>(parameter));
    }
}

MotorGroup::MotorGroup(MotorGroupParameters parameters, std::initializer_list<int> ports)
{
    for (auto port : ports) {
        MotorParameters motor_parameters = {
            port: port,
            reversed: parameters.reversed,
            brake_mode: parameters.brake_mode,
            ratio: parameters.ratio
        };

        motors.push_back(std::make_unique<Motor>(motor_parameters));
    }
}

void MotorGroup::move(double voltage)
{
    for (int i = 0; i < (int)motors.size(); i++) {
        motors[i]->move(voltage);
    }
}

void MotorGroup::move_velocity(double velocity)
{
    for (int i = 0; i < (int)motors.size(); i++) {
        motors[i]->move_velocity(velocity);
    }
}

double MotorGroup::get_velocity()
{
    double total_velocity = 0;
    for (int i = 0; i < (int)motors.size(); i++) {
        total_velocity += motors[i]->get_velocity();
    }

    return total_velocity / (double)motors.size();
}

void MotorGroup::set_brake_mode(MotorBrakeMode brake_mode)
{
    for (int i = 0; i < (int)motors.size(); i++) {
        motors[i]->set_brake_mode(brake_mode);
    }
}

MotorBrakeMode MotorGroup::get_brake_mode()
{
    return motors[0]->get_brake_mode();
}

void MotorGroup::set_reversed(bool reversed)
{
    for (int i = 0; i < (int)motors.size(); i++) {
        motors[i]->set_reversed(reversed);
    }
}

bool MotorGroup::get_reversed()
{
    return motors[0]->get_reversed();
}

double MotorGroup::get_temp()
{
    double max = motors[0]->get_temp();

    for (int i = 1; i < (int)motors.size(); i++) {
        double temp = motors[i]->get_temp();

        if (temp > max) {
            max = temp;
        }
    }

    return max;
}

void MotorGroup::set_ratio(double ratio)
{
    for (int i = 0; i < (int)motors.size(); i++) {
        motors[i]->set_ratio(ratio);
    }
}

double MotorGroup::get_ratio()
{
    return motors[0]->get_ratio();
}

bool MotorGroup::is_installed()
{
    bool installed = true;

    for (int i = 0; i < (int)motors.size(); i++) {
        if (!motors[i]->is_installed()) 
        {
            installed = false;
            break;
        }
    }

    return installed;
}

std::vector<int> MotorGroup::get_uninstalled_motors()
{
    std::vector<int> uninstalled = std::vector<int>();

    for (int i = 0; i < (int)motors.size(); i++) {
        if (!motors[i]->is_installed()) 
        {
            uninstalled.push_back(motors[i]->get_port());
        }
    }

    return uninstalled;
}

#endif