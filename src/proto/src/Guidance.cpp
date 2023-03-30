#include "Guidance.h"

Commands Guidance::get_commanded_state(const KinematicState& estimated_state, double time);
{
    return this->commands;
}

SimpleAscent::SimpleAscent()
{
    this->commands.z_axis.zero();
    this->commands.z_axis.data[2] = 1.0;
}

Commands SimpleAscent::get_commanded_state(const KinematicState& estimated_state, double time)
{
    if(this->chute->is_deployed())
    {
        return this->commands;
    }
    else if (estimated_state.velocity.z < -1.0)
    {
        this->chute->deploy(time);
        this->commands.force.zero();
        this->commands.torque.zero();
        return this->commands;
    }

    return this->commands;
}

Commands VerticalAscent::get_commanded_state(const KinematicState& estimated_state, double time)
{
    if(this->chute->is_deployed())
    {
        return this->commands;
    }
    else if (estimated_state.velocity.z < -1.0)
    {
        this->chute->deploy(time);
        this->commands.force.zero();
        this->commands.torque.zero();
        return this->commands;
    }

    //Vector velocity_correction_arm(-estimated_state.velocity.y,estimated_state.velocity.x,0.0);

    double x_scaled = this->damping*estimated_state.acceleration.x - this->proportional*estimated_state.velocity.x;
    double y_scaled = this->damping*estimated_state.acceleration.y - this->proportional*estimated_state.velocity.y;

    Vector rescaled_anti_vector(x_scaled,y_scaled,estimated_state.velocity.z);

    this->commands.axis_orientation = rescaled_anti_vector * (1.0/rescaled_anti_vector.norm());

    return this->commands;
}
