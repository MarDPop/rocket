#include "../include/Guidance.h"

#include "../include/Parachute.h"

Guidance::Guidance(){
    this->commands.z_axis.zero();
    this->commands.z_axis.data[2] = 1.0;
}

Guidance::~Guidance(){}

const Commands& Guidance::get_commands(const KinematicState& estimated_state, double time)
{
    return this->commands;
}

GuidanceSimpleAscent::GuidanceSimpleAscent() : Guidance()
{
}

const Commands& GuidanceSimpleAscent::get_commands(const KinematicState& estimated_state, double time)
{
    if(this->chute->is_deployed())
    {
        return this->commands;
    }
    else if (estimated_state.velocity.z < -1.0)
    {
        this->chute->deploy(time);
        this->commands.z_axis.zero();
        return this->commands;
    }

    return this->commands;
}

GuidanceVerticalAscent::GuidanceVerticalAscent(){}

const Commands& GuidanceVerticalAscent::get_commands(const KinematicState& estimated_state, double time)
{
    if(this->chute->is_deployed())
    {
        return this->commands;
    }
    else if (estimated_state.velocity.z < -10.0)
    {
        this->chute->deploy(time);
        this->commands.z_axis.zero();
        return this->commands;
    }

    //Vector velocity_correction_arm(-estimated_state.velocity.y,estimated_state.velocity.x,0.0);

    double x_scaled = this->damping*estimated_state.acceleration.x - this->proportional*estimated_state.velocity.x;
    double y_scaled = this->damping*estimated_state.acceleration.y - this->proportional*estimated_state.velocity.y;

    Vector rescaled_anti_vector(x_scaled,y_scaled,estimated_state.velocity.z);

    double scale = rescaled_anti_vector.norm();

    if(scale > 1e-4)
    {
        this->commands.z_axis = rescaled_anti_vector * (1.0/rescaled_anti_vector.norm());
    }
    else
    {
        this->commands.z_axis.zero();
        this->commands.z_axis.data[2] = 1.0;
    }

    return this->commands;
}
