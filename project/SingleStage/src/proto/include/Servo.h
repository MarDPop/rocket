#pragma once

#include <algorithm>
#include <array>

class Servo
{
protected:

    double _commanded_angle = 0;

    double _angle = 0;

    double _torque = 0;

public:

    inline Servo() {}
    virtual ~Servo();

    inline virtual void set_commanded_angle(double commanded_angle)
    {
        this->_commanded_angle = commanded_angle;
    }

    inline virtual void set_commanded_voltage(double commanded_voltage)
    {
        this->_commanded_angle = commanded_voltage;
    }

    inline double get_angle() const
    {
        return this->_angle;
    }

    inline void set_torque(double torque)
    {
        this->_torque = torque;
    }

    virtual void update(double time)
    {
        this->_angle = this->_commanded_angle;
    }
};

class BoundedServo : public virtual Servo
{
    const double _min_angle;

    const double _max_angle;

public:

    BoundedServo(double min_angle, double max_angle);
    virtual ~BoundedServo();

    inline void set_commanded_angle(double commanded_angle) override
    {
        this->_commanded_angle = std::clamp(commanded_angle,this->_min_angle,this->_max_angle);
    }
};

class SlewingServo : public virtual Servo
{
    double _min_angle;

    double _max_angle;

    double _voltage_to_angle;

    double _max_torque;

    double _slew_rate;

    double _time_ref;

public:

    SlewingServo();
    virtual ~SlewingServo();

    inline void set_performance(double min_angle,
                                double max_angle,
                          double voltage_to_angle,
                          double max_torque,
                          double slew_rate)
    {
        this->_min_angle = min_angle;
        this->_max_angle = max_angle;
        this->_voltage_to_angle = voltage_to_angle;
        this->_max_torque = max_torque;
        this->_slew_rate = slew_rate;
    }

    inline void set_commanded_angle(double commanded_angle) override
    {
        this->_commanded_angle = std::clamp(commanded_angle,this->_min_angle,this->_max_angle);
    }

    inline void set_commanded_voltage(double commanded_voltage) override
    {
        this->set_commanded_angle(commanded_voltage*this->_voltage_to_angle);
    }

    void update(double time);
};
