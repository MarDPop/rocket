#pragma once

#include "Kinematics.h"
#include "Aerodynamics.h"
#include "Guidance.h"

class Control
{

public:

    Control();
    virtual ~Control();

    virtual void get_outputs(const Commands& commands, const KinematicState& estimated_state, double time);

};

template <unsigned NUMBER_FINS>
class FinControlSimple : public virtual Control
{
    double fin_gain = 1.0;

    double proportional = 1.0;

    double damping = 1.0;

    Vector get_desired_arm_magnitude_body(const Commands& commands, const KinematicState& estimated_state);

    FinControlAero<NUMBER_FINS>& aero;

public:

    FinControlSimple(const FinControlAero<NUMBER_FINS>& _aero);
    virtual ~FinControlSimple();

    inline void set_fin_gain(double fin_gain)
    {
        this->fin_gain = fin_gain;
    }

    inline void set_controller_values(double proportional, double damping)
    {
        this->proportional = proportional;
        this->damping = damping;
    }

    void get_outputs(const Commands& commands, const KinematicState& estimated_state, double time) override;
};
