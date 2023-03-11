#pragma once

#include <array>

class Servo
{
    std::array<double,2> angle_range;

    double max_torque;

    double slew_rate;

    double voltage_to_angle;

    double commanded_angle;

    double angle;

    double time_ref;

public:

    Servo();
    virtual ~Servo();

    inline void set_angle_range(std::array<double,2> _angle_range)
    {
        this->angle_range = _angle_range;
    }

    inline void set_max_torque(double _max_torque)
    {
        this->max_torque = _max_torque;
    }

    inline void set_slew_rate(double _slew_rate)
    {
        this->slew_rate = _slew_rate;
    }

    inline void set_commanded_angle(double _commanded_angle)
    {
        this->command_angle = _commanded_angle;
    }

    inline void set_commanded_voltage(double _commanded_voltage)
    {
        this->command_angle = _commanded_voltage*this->voltage_to_angle;
    }

    inline double get_angle()
    {
        return this->angle;
    }

    void update(double time);
};
