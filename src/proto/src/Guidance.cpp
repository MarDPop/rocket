#include "Guidance.h"

Commands Guidance::get_commanded_state(const KinematicState& estimated_state, double time);
{
    return this->commands;
}

SimpleAscent::SimpleAscent(){}

Commands SimpleAscent::get_commanded_state(const KinematicState& estimated_state, double time)
{
    if(this->chute->is_deployed())
    {
        return this->commands;
    }
    else if (estimated_state.velocity.z < -1.0)
    {
        this->chute->deploy();
        this->commands.force.zero();
        this->commands.torque.zero();
        return this->commands;
    }

    Commands
    return this->commands;
}
