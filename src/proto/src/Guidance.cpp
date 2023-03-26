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
    //Vector orientation_correction_arm(-estimated_state.CS.z.y,estimated_state.CS.z.x,0.0);

    double inertial_arm_x = -this->proportionalVelocityConstant*estimated_state.velocity.y -
                                this->proportionalOrientationConstant*estimated_state.CS.z.y;
    double inertial_arm_y = this->proportionalVelocityConstant*estimated_state.velocity.x +
                                this->proportionalOrientationConstant*estimated_state.CS.z.x;
    Vector inertial_arm(inertial_arm_x,inertial_arm_y,0.0);

    this->commands.angular_velocity_in_body = estimated_state.CS*inertial_arm;

    return this->commands;
}
