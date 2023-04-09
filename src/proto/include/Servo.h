#pragma once

#include <algorithm>
#include <array>

class Servo
{
protected:

    double commanded_angle = 0;

    double angle = 0;

    double torque = 0;

public:

    inline Servo() {}
    virtual ~Servo() {}

    inline virtual void set_commanded_angle(double commanded_angle)
    {
        this->commanded_angle = commanded_angle;
    }

    inline virtual void set_commanded_voltage(double commanded_voltage)
    {
        this->commanded_angle = commanded_voltage;
    }

    inline double get_angle() const
    {
        return this->angle;
    }

    inline void set_torque(double torque)
    {
        this->torque = torque;
    }

    virtual void update(double time)
    {
        this->angle = this->commanded_angle;
    }
};

class SimpleServo : public virtual Servo
{
    std::array<double,2> angle_range;

    double voltage_to_angle;

    double max_torque;

    double slew_rate;

    double time_ref;

public:

    SimpleServo();
    virtual ~SimpleServo();

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

    inline void set_commanded_angle(double commanded_angle) override
    {
        this->commanded_angle = std::clamp(commanded_angle,this->angle_range[0],this->angle_range[1]);
    }

    inline void set_commanded_voltage(double commanded_voltage) override
    {
        this->set_commanded_angle(commanded_voltage*this->voltage_to_angle);
    }

    void update(double time);
};
