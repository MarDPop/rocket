#include "../include/SingleStageRocket.h"

#include "../../common/include/util.h"
#include "../include/Action.h"
#include <fstream>
#include <numeric>
#include <iostream>
#include <exception>

void SingleStageRocket::update_inertia()
{
    const Inertia_Basic& fuel_inertia = this->thruster->get_inertia();

    // Get mass first
    this->inertia.mass = this->inertia_empty.mass + fuel_inertia.mass;

    // Compute center of mass
    this->inertia.CoM = this->inertia_empty.CoM*this->inertia_empty.mass;
    // Fuel inertial can be assumed to be in axial
    this->inertia.CoM.z += fuel_inertia.CoM_axial*fuel_inertia.mass;
    this->inertia.CoM *= (1.0/this->inertia.mass);

    // ASSUMPTION: offset CoM contributes negligibly to inertia

    // Off axial terms are just from the structure
    // no parallel axis theory for now, should be neglible
    this->inertia.Ixy = this->inertia_empty.Ixy;
    this->inertia.Ixz = this->inertia_empty.Ixz;
    this->inertia.Iyz = this->inertia_empty.Iyz;

    // Axial terms can be added directly
    this->inertia.Izz = this->inertia_empty.Izz + fuel_inertia.Izz;

    double CoM_z_fuel = fuel_inertia.CoM_axial - this->inertia.CoM.z;
    double CoM_z_struct = this->inertia_empty.CoM.z - this->inertia.CoM.z;
    double parallel_axis_Z = fuel_inertia.mass*CoM_z_fuel*CoM_z_fuel + this->inertia_empty.mass*CoM_z_struct*CoM_z_struct;

    this->inertia.Ixx = this->inertia_empty.Ixx + fuel_inertia.Ixx + parallel_axis_Z;
    this->inertia.Iyy = this->inertia_empty.Iyy + fuel_inertia.Ixx + parallel_axis_Z;
}

void SingleStageRocket::compute_acceleration(double time)
{
    this->_atmosphere->set(this->state.position.z, time);

    BodyAction allActions;
    allActions.location = this->inertia.CoM;

    allActions += this->aerodynamics->update();

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
    this->state.acceleration.z -= this->_atmosphere->values.gravity;

    // TODO: explore doing in body frame
    Axis body2inertial = this->state.CS.get_transpose();
    // rotate Inertia to inertial frame
    Axis I_inertial = body2inertial * this->inertia.get_inertia_matrix() * this->state.CS; // See InertiaTensor.pdf
    // rotate moment to inertial frame
    Vector total_moment = body2inertial * allActions.moment;
    this->state.angular_acceleration = I_inertial.get_inverse() * total_moment;
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

    Vector inertial_rotation = this->state.angular_velocity*dt;
    this->state.angular_velocity += this->state.angular_acceleration*dt;

    // Rotations are non linear -> rotate CS
    double angle = inertial_rotation.norm();
    if(angle > 1e-6)
    {
        inertial_rotation *= (1.0/angle);
        this->state.CS = Axis(angle,inertial_rotation)*state0.CS;
    }

    // Compute mass changes, no need to recompute at next step
    if(this->thruster->is_active())
    {
        this->thruster->set_time(time);
        this->update_inertia();
    }

    /* Huen step */

    // recompute state rate at time + dt
    this->compute_acceleration(time);

    double dt_half = dt*0.5;

    this->state.position = state0.position + (state0.velocity + this->state.velocity)*dt_half;
    this->state.velocity = state0.velocity + (state0.acceleration + this->state.acceleration)*dt_half;

    this->state.angular_velocity = state0.angular_velocity + (state0.angular_acceleration + this->state.angular_acceleration)*dt_half;

    // Average angular rate
    inertial_rotation = (state0.angular_velocity + this->state.angular_velocity)*dt_half;
    angle = inertial_rotation.norm();
    if(angle > 1e-6)
    {
        inertial_rotation *= (1.0/angle);
        this->state.CS = Axis(angle,inertial_rotation)*state0.CS;  // confirmed true since rotation matrix is orthogonal
        this->state.CS.gram_schmidt_orthogonalize();
    }
    else
    {
        this->state.CS = state0.CS;
    }
}
SingleStageRocket::SingleStageRocket(Atmosphere* atmosphere) : gnc(*this), _atmosphere(atmosphere) {}

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
    this->update_inertia();

    this->_atmosphere->set(0.0,0.0);
    this->gnc.navigation.sensors->calibrate(0.0,this->_atmosphere->values.gravity,this->_atmosphere->values.temperature,this->_atmosphere->values.pressure);

    this->gnc.navigation.filter->init(this->state,0.0);

}

