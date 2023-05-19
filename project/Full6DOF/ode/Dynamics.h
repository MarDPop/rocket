#pragma once

#include <array>

template<unsigned NSTATES>
struct ODE_Dynamics 
{
    virtual void set_state(const std::array<double, NSTATES>& x, const double& time, std::array<double,NSTATES>& x) = 0;

    virtual bool stop() = 0;
};