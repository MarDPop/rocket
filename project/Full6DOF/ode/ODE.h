#pragma once

#include "Dynamics.h"

#include <memory>
#include <array>

struct ODE_Options
{
    double step_size;
    unsigned max_number_steps;
};

template<unsigned NSTATES>
class ODE 
{
    std::array<double, NSTATES> _state;

    std::array<double, NSTATES> _state_rate;

    double _time;

    ODE_Options _options;

    ODE_Dynamics& _dynamics;

    void (ODE::*step)();

public:

    ODE(ODE_Dynamics& dynamics) : _dynamics(dynamics) {}

    virtual ~ODE(){}

    inline void set_options(const ODE_Options& options) 
    {
        this->_options = options;
    }

    inline void set_state_and_time(const std::array<double, NSTATES>& state, const double& time)
    {
        this->_state = state;
        this->_time = time;
    }

    inline void run_to_time(double time)
    {
        while(this->_time < time && !_dynamics.stop()) 
        {
            this->step();
        }
    }
};