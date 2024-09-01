#include "pneumatic.hpp"

using namespace lib15442c;

#ifndef LIB15442C_MOCK_DEVICES_ONLY
Pneumatic::Pneumatic(char adi_port, bool default_state, bool extended_state) :
    extender_port(INTERNAL_ADI_PORT), adi_port(adi_port), default_state(default_state), extended_state(extended_state)
{
    pros::c::ext_adi_port_set_config(INTERNAL_ADI_PORT, adi_port, pros::adi_port_config_e::E_ADI_DIGITAL_OUT);

    if (extended_state == true)
    {
        set_value(default_state);
    }
    else
    {
        set_value(!default_state);
    }
}

Pneumatic::Pneumatic(int extender_port, char adi_port, bool default_state, bool extended_state) :
    extender_port(extender_port), adi_port(adi_port), default_state(default_state), extended_state(extended_state)
{
    pros::c::ext_adi_port_set_config(extender_port, adi_port, pros::adi_port_config_e::E_ADI_DIGITAL_OUT);

    if (extended_state == true)
    {
        set_value(default_state);
    }
    else
    {
        set_value(!default_state);
    }
}

void Pneumatic::set_value(bool value)
{
    pros::c::ext_adi_port_set_value(extender_port, adi_port, value);
}

void Pneumatic::extend()
{
    set_value(extended_state);
}

void Pneumatic::retract()
{
    set_value(!extended_state);
}

void Pneumatic::toggle()
{
    set_value(!get_value());
}

bool Pneumatic::get_value()
{
    return pros::c::ext_adi_port_get_value(extender_port, adi_port) == extended_state;
}

PneumaticGroup::PneumaticGroup(std::vector<std::shared_ptr<Pneumatic>> pneumatics) :
    pneumatics(pneumatics)
{};

void PneumaticGroup::set_value(bool value)
{
    for(auto pneumatic : pneumatics)
    {
        pneumatic->set_value(value);
    }
}

void PneumaticGroup::extend()
{
    for(auto pneumatic : pneumatics)
    {
        pneumatic->extend();
    }
}

void PneumaticGroup::retract()
{
    for(auto pneumatic : pneumatics)
    {
        pneumatic->retract();
    }
}

void PneumaticGroup::toggle()
{
    for(auto pneumatic : pneumatics)
    {
        pneumatic->toggle();
    }
}

bool PneumaticGroup::get_value()
{
    return pneumatics[0]->get_value();
}

std::vector<bool> PneumaticGroup::get_all_values()
{
    std::vector<bool> values;
    for(auto pneumatic : pneumatics)
    {
        values.push_back(pneumatic->get_value());
    }
    return values;
}

#endif

MockPneumatic::MockPneumatic(bool default_state)
    : current_state(default_state) {};

void MockPneumatic::set_value(bool value)
{
    current_state = value;
}

void MockPneumatic::extend()
{
    set_value(true);
}

void MockPneumatic::retract()
{
    set_value(false);
}

void MockPneumatic::toggle()
{
    set_value(!get_value());
}

bool MockPneumatic::get_value()
{
    return current_state;
}