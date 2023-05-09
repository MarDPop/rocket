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

    if(this->principal_axis_assumption)
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
    this->state.acceleration.z -= this->_environment->values.gravity;

    Vector angular_velocity_body = this->state.CS*this->state.angular_velocity;

    // Don't do extra work for on principal axis
    Vector angular_acceleration_body;
    if(this->principal_axis_assumption)
    {
        double Iw[3];
        Iw[0] = (this->inertia.MoI.Izz - this->inertia.MoI.Iyy)*angular_velocity_body.z*angular_velocity_body.y;
        Iw[1] = (this->inertia.MoI.Ixx - this->inertia.MoI.Izz)*angular_velocity_body.z*angular_velocity_body.x;
        Iw[2] = (this->inertia.MoI.Iyy - this->inertia.MoI.Ixx)*angular_velocity_body.y*angular_velocity_body.x;

        angular_acceleration_body.data[0] = (allActions.moment.data[0] - Iw[0])/this->inertia.MoI.I[0];
        angular_acceleration_body.data[1] = (allActions.moment.data[1] - Iw[1])/this->inertia.MoI.I[1];
        angular_acceleration_body.data[2] = (allActions.moment.data[2] - Iw[2])/this->inertia.MoI.I[2];
    }
    else
    {
        Axis I = this->inertia.MoI.get_inertia_matrix();
        Vector torque = allActions.moment - angular_velocity_body.cross(I*angular_velocity_body);

        if(this->thruster->is_active())
        {
            Axis I_dot = this->MoI_rate_change.get_inertia_matrix();
            torque -= I_dot*angular_velocity_body;
        }
        // remember angular acceleration is in body frame
        angular_acceleration_body = this->I_inverse * torque;
    }

    this->state.angular_acceleration = this->state.CS.transpose_mult(angular_acceleration_body);
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

