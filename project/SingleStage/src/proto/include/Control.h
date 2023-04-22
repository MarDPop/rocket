#pragma once

#include "Kinematics.h"
#include "Aerodynamics.h"
#include "Guidance.h"
#include "Sensors.h"

class Control
{

public:

    Control();
    virtual ~Control();

    virtual void get_outputs(const Commands& commands, const KinematicState& estimated_state, const Sensors* sensors, double time);

};

class ControlFinSimple : public virtual Control
{
    FinControlAero& aero; // can I make constant?

    double fin_gain = 1.0;

    double proportional = 1.0;

    double damping = 1.0;

    Vector get_desired_arm_magnitude_body(const Commands& commands, const KinematicState& estimated_state);

public:

    ControlFinSimple(FinControlAero& _aero);
    virtual ~ControlFinSimple();

    inline void set_fin_gain(double fin_gain)
    {
        this->fin_gain = fin_gain;
    }

    inline void set_controller_values(double proportional, double damping)
    {
        this->proportional = proportional;
        this->damping = damping;
    }

    void get_outputs(const Commands& commands, const KinematicState& estimated_state, const Sensors* sensors, double time) override;
};
