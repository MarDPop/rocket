#pragma once

#include "Kinematics.h"

#include "Servo.h"

#include "Guidance.h"

class Control
{

public:

    Control();
    virtual ~Control();

    virtual void get_outputs(const Commands& commands, const KinematicState& estimated_state, double time);

};

template <unsigned NUMBER_FINS>
class FinControl : public virtual Control
{
    double theta_gain = 0.0;

public:

    std::array<Servo, NUMBER_FINS> servos;

    FinControl();
    virtual ~FinControl();

    void set_theta_gain(double theta_gain)
    {
        this->theta_gain = theta_gain;
    }

    virtual void get_outputs(const Commands& commands, const KinematicState& estimated_state, double time);
};
