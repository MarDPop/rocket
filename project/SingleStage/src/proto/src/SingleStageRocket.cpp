#include "../include/SingleStageRocket.h"

#include "../../common/include/util.h"
#include "../include/Action.h"
#include <fstream>
#include <numeric>
#include <iostream>
#include <exception>

void SingleStageRocket::update_inertia(double inv_dt)
{
    const Inertia_Basic& fuel_inertia = this->thruster->get_inertia();

    this->MoI_rate_change = this->inertia.MoI;

    if(this->symmetric_inertia_assumption)
    {
        // Get mass first
        this->inertia.mass = this->inertia_empty.mass + fuel_inertia.mass;

        // Compute center of mass
        this->inertia.CoM = this->inertia_empty.CoM*this->inertia_empty.mass;
        // Fuel inertial can be assumed to be in axial
        this->inertia.CoM.z += fuel_inertia.CoM_axial*fuel_inertia.mass;
        this->inertia.CoM *= (1.0/this->inertia.mass);

        // Axial terms can be added directly
        this->inertia.MoI.Izz = this->inertia_empty.MoI.Izz + fuel_inertia.Izz;

        double CoM_z_fuel = fuel_inertia.CoM_axial - this->inertia.CoM.z;
        double CoM_z_struct = this->inertia_empty.CoM.z - this->inertia.CoM.z;
        double parallel_axis_Z = fuel_inertia.mass*CoM_z_fuel*CoM_z_fuel + this->inertia_empty.mass*CoM_z_struct*CoM_z_struct;

        this->inertia.MoI.Ixx = this->inertia_empty.MoI.Ixx + fuel_inertia.Ixx + parallel_axis_Z;
        this->inertia.MoI.Iyy = this->inertia_empty.MoI.Iyy + fuel_inertia.Ixx + parallel_axis_Z;

        this->I_inverse.data[0] = 1.0/this->inertia.MoI.Ixx;
        this->I_inverse.data[4] = 1.0/this->inertia.MoI.Iyy;
        this->I_inverse.data[8] = 1.0/this->inertia.MoI.Izz;
    }
    else
    {
        Inertia I_fuel;
        I_fuel.set_from_basic(fuel_inertia);
        this->inertia = this->inertia_empty + I_fuel;
        // could improve speed here
        this->I_inverse = this->inertia.MoI.get_inertia_matrix().get_inverse();
    }

    for(unsigned i = 0; i < 6; i++)
    {
        this->MoI_rate_change.I[i] = (this->inertia.MoI.I[i] - this->MoI_rate_change.I[i])*inv_dt;
    }
}

void SingleStageRocket::compute_acceleration(double time)
{
    this->_environment->set(this->state.position.z, time);

    BodyAction allActions;
    allActions.location = this->inertia.CoM;
    allActions.force.zero();
    allActions.moment.zero();

    //allActions += this->aerodynamics->update();

    if(this->thruster->is_active())
    {
        allActions += this->thruster->get_action();
    }

    if(this->parachute->is_deployed())
    {
        allActions += this->parachute->update(time);
    }

    Vector total_force = this->state.CS.transpose_mult(allActions.force);

    this->state.acceleration = total_force * (1.0/this->inertia.mass);
    this->state.acceleration.z -= this->_environment->values.gravity;

    // Don't do extra work for symmetric
    Axis I = this->inertia.MoI.get_inertia_matrix();
    Vector torque = allActions.moment - this->state.angular_velocity.cross(I*this->state.angular_velocity);

    if(this->thruster->is_active())
    {
        Axis I_dot = this->MoI_rate_change.get_inertia_matrix();
        torque -= I_dot*this->state.angular_velocity;
    }
    // remember angular acceleration is in body frame
    this->state.angular_acceleration = this->I_inverse * torque;
}

void SingleStageRocket::step(double& time, double dt)
{
    // Get initial state
    this->compute_acceleration(time);

    // only update GNC at beginning of time step
    this->gnc.update(time); // TODO: Investigate why this breaks things in dynamics when put at start of step

    KinematicState state0 = this->state;

    // propagate to time + dt
    time += dt;

    this->state.position += this->state.velocity*dt;
    this->state.velocity += this->state.acceleration*dt;

    Vector inertial_rotation_rate = state0.CS.transpose_mult(this->state.angular_velocity);
    this->state.angular_velocity += this->state.angular_acceleration*dt;

    // Rotations are non linear -> rotate CS
    double rotation_rate = inertial_rotation_rate.norm();
    if(rotation_rate > 1e-8)
    {
        this->state.CS = Axis(rotation_rate*dt,inertial_rotation_rate*(1.0/rotation_rate))*state0.CS;  // confirmed true since rotation matrix is orthogonal
    }

    // Compute mass changes, no need to recompute at next step
    if(this->thruster->is_active())
    {
        this->thruster->set_time(time);
        this->update_inertia(1.0/dt);
    }

    /* Huen step */

    // recompute state rate at time + dt
    this->compute_acceleration(time);

    double dt_half = dt*0.5;

    this->state.position = state0.position + (state0.velocity + this->state.velocity)*dt_half;
    this->state.velocity = state0.velocity + (state0.acceleration + this->state.acceleration)*dt_half;

    this->state.angular_velocity = state0.angular_velocity + (state0.angular_acceleration + this->state.angular_acceleration)*dt_half;

    // Average angular rate
    inertial_rotation_rate += this->state.CS.transpose_mult(this->state.angular_velocity);
    rotation_rate = inertial_rotation_rate.norm();
    if(rotation_rate > 1e-8)
    {
        this->state.CS = Axis(rotation_rate*dt_half,inertial_rotation_rate*(1.0/rotation_rate))*state0.CS;  // confirmed true since rotation matrix is orthogonal
    }
    else
    {
        this->state.CS = state0.CS;
    }
}
SingleStageRocket::SingleStageRocket(Environment* atmosphere) : gnc(*this), _environment(atmosphere) {}

SingleStageRocket::~SingleStageRocket(){}

void SingleStageRocket::init(double launch_angle, double launch_heading)
{
    this->state.position.zero();
    this->state.velocity.zero();
    this->state.acceleration.zero();
    this->state.angular_velocity.zero();
    this->state.angular_acceleration.zero();

    double cphi = cos(launch_heading);
    double sphi = sin(launch_heading);
    double stheta = sin(1.5707963267948966192 - launch_angle);
    double ctheta = sqrt(1 - stheta*stheta);

    // z rotation by launch_heading
    // y rotation by launch_angle
    // y axis points north at zero heading
    this->state.CS.axis.y.x = sphi;
    this->state.CS.axis.y.y = cphi;
    this->state.CS.axis.y.z = 0.0;
    this->state.CS.axis.z.x = ctheta*cphi;
    this->state.CS.axis.z.y = ctheta*-sphi;
    this->state.CS.axis.z.z = stheta;
    Vector::cross(this->state.CS.axis.y,this->state.CS.axis.z,this->state.CS.axis.x);

    this->thruster->set_time(0.0);
    this->update_inertia(0);

    this->_environment->set(0.0,0.0);
    this->gnc.navigation.sensors->calibrate(0.0,this->_environment->values.gravity,this->_environment->values.temperature,this->_environment->values.pressure);

    this->gnc.navigation.filter->init(this->state,0.0);

}

