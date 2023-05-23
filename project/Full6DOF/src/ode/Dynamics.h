#pragma once

#include <array>

/**
 * @brief a class which defines behavior for dynamics which can be implemented by an ODE
 * 
 * @tparam NSTATES 
 */
template<unsigned NSTATES>
struct ODE_Dynamics 
{
    /**
     * @brief this method describes the rate of change to a state given it's current state and time
     * 
     * @param x current state
     * @param time current time
     * @param dx rate of change of state
     * @return true if state is valid
     * @return false if state is invalid and ODE should stop
     */
    virtual bool set_state(const std::array<double, NSTATES>& x, const double& time, std::array<double,NSTATES>& dx) = 0;
};