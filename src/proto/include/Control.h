#pragma once

#include "Kinematics.h"

#include "Servo.h"

class Control
{

public:

    Control();
    virtual ~Control();

    virtual void get_outputs(const KinematicState& commanded_state, const KinematicState& estimated_state, double time) = 0;

};

class FinControl : public virtual Control
{

};
