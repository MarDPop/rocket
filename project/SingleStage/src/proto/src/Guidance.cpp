#include "../include/Guidance.h"

#include "../include/Parachute.h"

Guidance::Guidance()
{
    this->_commands.z_axis_inertial.zero();
    this->_commands.z_axis_inertial.data[2] = 1.0;
}

Guidance::~Guidance(){}

const Commands& Guidance::get_commands(const KinematicState& estimated_state, double time)
{
    return this->_commands;
}

GuidanceTimedParachute::GuidanceTimedParachute(double burnout, double delay) : Guidance(), _deploy(burnout + delay) {}

const Commands& GuidanceTimedParachute::get_commands(const KinematicState& estimated_state, double time)
{
    if( !this->chute->is_deployed() && time > this->_deploy )
    {
        this->_commands.z_axis_inertial.zero();
        this->chute->deploy(time);
    }
    else if( estimated_state.velocity.z < -10.0 ) // TODO: Fix
    {
        this->_commands.z_axis_inertial.zero();
    }

    return this->_commands;
}

GuidanceSimpleAscent::GuidanceSimpleAscent() : Guidance() {}

const Commands& GuidanceSimpleAscent::get_commands(const KinematicState& estimated_state, double time)
{
    if (!this->chute->is_deployed() && estimated_state.velocity.z < -1.0)
    {
        this->chute->deploy(time);
        this->_commands.z_axis_inertial.zero();
    }
    return this->_commands;
}

GuidanceVerticalAscent::GuidanceVerticalAscent(){}

const Commands& GuidanceVerticalAscent::get_commands(const KinematicState& estimated_state, double time)
{
    if(this->chute->is_deployed())
    {
        return this->_commands;
    }
    else if (estimated_state.velocity.z < -30.0)
    {
        this->chute->deploy(time);
        this->_commands.z_axis_inertial.zero();
        return this->_commands;
    }

    double x_scaled = -this->damping*estimated_state.acceleration.x - this->proportional*estimated_state.velocity.x;
    double y_scaled = -this->damping*estimated_state.acceleration.y - this->proportional*estimated_state.velocity.y;

    Vector rescaled_anti_vector(x_scaled,y_scaled,estimated_state.velocity.z);

    double scale = rescaled_anti_vector.norm();

    if(scale > 1e-4) // REMEMBER TO PUT BACK
    {
        this->_commands.z_axis_inertial = rescaled_anti_vector * (1.0/scale);
    }
    else
    {
        this->_commands.z_axis_inertial.zero();
        this->_commands.z_axis_inertial.data[2] = 1.0;
    }

    return this->_commands;
}
