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
    double fin_gain = 1.0;

    double proportional = 1.0;

    double damping = 1.0;

    double integration = 1.0;

    double saturation_level = 1.0;

    double integral_sum = 0;

    double old_time = 0;

    Vector get_desired_arm_magnitude(const Commands& commands, const KinematicState& estimated_state, double time);

public:

    std::array<Servo, NUMBER_FINS> servos;

    FinControl();
    virtual ~FinControl();

    inline void set_fin_gain(double fin_gain)
    {
        this->fin_gain = gain;
    }

    inline void set_controller_values(double proportional, double damping, double integration, double saturation_level)
    {
        this->proportional = proportional;
        this->damping = damping;
        this->integration = integration;
        this->saturation_level = saturation_level;
    }

    inline void reset_integrator()
    {
        this->integral_sum = 0;
    }

    inline void reset_time()
    {
        this->old_time = 0;
    }

    virtual void get_outputs(const Commands& commands, const KinematicState& estimated_state, double time);
};
