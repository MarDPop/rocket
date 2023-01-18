#include "../include/SingleStageRocket.h"

#include "../../common/include/util.h"
#include <fstream>
#include <numeric>
#include <iostream>

void SingleStageRocket::compute_acceleration(double time)
{
    this->altitude_table.set(this->state.position.z, time);

    this->aerodynamics.update();

    this->state.acceleration = this->aerodynamics.force;
    this->state.angular_acceleration = this->aerodynamics.moment;

    if(this->not_empty)
    {
        this->thruster.set(this->altitude_table.values->pressure);
        this->state.acceleration += this->state.CS.axis.z * this->thruster.thrust;
    }

    if(this->control)
    {
        this->control->update(time);

        this->state.acceleration += this->control->dForce;
        this->state.angular_acceleration += this->control->dMoment;
    }

    this->state.acceleration *= (1.0/this->inertia.mass);
    this->state.acceleration.z -= this->altitude_table.values->gravity;

    Axis I_inertial = this->state.CS.get_transpose(); // might have to transpose CS
    int i = 0;
    for(; i < 6;i++)
    {
        I_inertial.data[i] *= this->inertia.Ixx;
    }
    for(; i < 9;i++)
    {
        I_inertial.data[i] *= this->inertia.Izz;
    }
    //Vector Iw = I_inertial * this->angular_velocity;
    //Vector rhs = this->angular_acceleration - this->angular_velocity.cross(Iw);
    // this->angular_acceleration = I_inertial.get_inverse() * rhs;
    this->state.angular_acceleration = I_inertial.get_inverse() * this->state.angular_acceleration;
}

void SingleStageRocket::step(double& time, double dt)
{
    // Get initial state
    this->compute_acceleration(time);

    KinematicState state0 = this->state;

    // propagate to time + dt
    time += dt;

    this->state.position += this->state.velocity*dt;
    this->state.velocity += this->state.acceleration*dt;
    Vector angle_axis = this->state.angular_velocity*dt;
    this->state.angular_velocity += this->state.angular_acceleration*dt;

    // Rotations are non linear -> rotate CS
    double angle = angle_axis.norm();
    if(angle > 1e-6) {
        angle_axis *= (1.0/angle);
        this->state.CS = Axis(angle,angle_axis)*state0.CS;
    }

    // Compute mass changes, no need to recompute at next step
    if(this->not_empty) {
        this->inertia.mass -= this->thruster.mass_rate*dt;
        this->get_inertia();
    }

    /* Huen step */

    // recompute state rate at time + dt
    this->compute_acceleration(time);

    double dt_half = dt*0.5;

    this->state.position = state0.position + (state0.velocity + this->state.velocity)*dt_half;
    this->state.velocity = state0.velocity + (state0.acceleration + this->state.acceleration)*dt_half;

    angle_axis = (state0.angular_velocity + this->state.angular_velocity)*dt_half;
    this->state.angular_velocity = state0.angular_velocity + (state0.angular_acceleration + this->state.angular_acceleration)*dt_half;

    angle = angle_axis.norm();
    if(angle > 1e-6)
    {
        angle_axis *= (1.0/angle);
        this->state.CS = Axis(angle,angle_axis)*state0.CS;
    }
}

void SingleStageRocket::get_inertia()
{
    double dm = this->inertia.mass - this->mass_empty;
    this->inertia.Ixx = this->I_empty[0] + this->dIdm[0]*dm;
    this->inertia.Izz = this->I_empty[1] + this->dIdm[1]*dm;
    this->inertia.COG = this->I_empty[2] + this->dIdm[2]*dm;

    this->not_empty = dm > 0.0;
}

SingleStageRocket::SingleStageRocket() : aerodynamics(*this)
{

}

SingleStageRocket::~SingleStageRocket(){}

void SingleStageRocket::init(double launch_angle, double launch_heading)
{
    memset(&this->state,0,sizeof(KinematicState));

    double cphi = cos(launch_heading);
    double sphi = sin(launch_heading);
    double stheta = sin(1.5707963267948966192 - launch_angle);
    double ctheta = sqrt(1 - stheta*stheta);

    // z rotation by launch_heading
    // y rotation by launch_angle
    // y axis points north at zero heading
    this->state.CS.axis.y.x = sphi;
    this->state.CS.axis.y.y = cphi;
    this->state.CS.axis.y.z = 0;
    this->state.CS.axis.z.x = ctheta*cphi;
    this->state.CS.axis.z.y = ctheta*-sphi;
    this->state.CS.axis.z.z = stheta;
    Vector::cross(this->state.CS.axis.y,this->state.CS.axis.z,this->state.CS.axis.x);

    this->inertia.mass = this->mass_full;
    this->get_inertia();
}

void SingleStageRocket::set_inertial_properties(double empty_mass, double full_mass, double I_empty[3], double I_full[3])
{
    this->mass_empty = empty_mass;
    this->mass_full = full_mass;
    this->I_empty[0] = I_empty[0]; // Ixx
    this->I_empty[1] = I_empty[1]; // Izz
    this->I_empty[2] = I_empty[2]; // COG
    double dm = full_mass - empty_mass;
    this->dIdm[0] = (I_full[0] - I_empty[0])/dm;
    this->dIdm[1] = (I_full[1] - I_empty[1])/dm;
    this->dIdm[2] = (I_full[2] - I_empty[2])/dm; // dCOG
}
