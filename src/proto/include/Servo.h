#pragma once

#include <array>

class Servo
{
    std::array<double,2> angle_range;

    double voltage_to_angle;

    double max_torque;

    double slew_rate;

    double commanded_angle;

    double angle = 0;

    double torque = 0;

    double time_ref;

public:

    Servo();
    virtual ~Servo();

    inline void set_performance(std::array<double,2> angle_range,
                          double voltage_to_angle,
                          double max_torque,
                          double slew_rate)
    {
        this->angle_range = angle_range;
        this->voltage_to_angle = voltage_to_angle;
        this->max_torque = max_torque;
        this->slew_rate = slew_rate;
    }

    inline void set_commanded_angle(double commanded_angle)
    {
        this->command_angle = std::clamp(commanded_angle, this->angle_range[0], this->angle_range[1]);
    }

    inline void set_commanded_voltage(double commanded_voltage)
    {
        this->set_commanded_angle(commanded_voltage*this->voltage_to_angle);
    }

    inline double get_angle()
    {
        return this->angle;
    }

    inline void set_torque(double torque)
    {
        this->torque = torque;
    }

    void update(double time);
};
