#pragma once

#include "Kinematics.h"

#include "Servo.h"

#include "Guidance.h"

class Control
{

public:

    Control();
    virtual ~Control();

    virtual void get_outputs(const Commands& commands, const KinematicState& estimated_state, double time) = 0;

};

template <unsigned NUMBER_FINS>
class FinControl : public virtual Control
{
    double pointing_gain;

    double velocity_gain;

public:

    std::array<Servo, NUMBER_FINS> servos;

    FinControl();
    virtual ~FinControl();

    void get_outputs(const Commands& commands, const KinematicState& estimated_state, double time);
};
