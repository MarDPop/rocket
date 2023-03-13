#include "../include/SingleStageRocket.h"

#include "../../common/include/util.h"
#include <fstream>
#include <numeric>
#include <iostream>

void SingleStageRocket::update_inertia()
{
    const auto& fuel_inertia = this->thruster->get_inertia();

    double cog_arm = inertia.COG - this->inertia.COG;

    this->inertia.mass = this->inertia_empty.mass + fuel_inertia.mass;
    this->inertia.Ixx = this->inertia_empty.Ixx + fuel_inertia.Ixx;
    this->inertia.Izz = this->inertia_empty.Izz + fuel_inertia.Izz + fuel_inertia.mass*cog_arm*cog_arm;
    this->inertia.COG = (this->inertia_empty.COG*this->inertia_empty.mass + fuel_inertia.COG*fuel_inertia.mass) / this->inertia.mass;
}

void SingleStageRocket::compute_acceleration(double time)
{
    this->altitude_table.set(this->state.position.z, time);

    this->gnc.update(time);

    this->aerodynamics->update();

    Vector total_force(this->aerodynamics->force);
    Vector total_moment(this->aerodynamics->moment);

    if(this->thruster->is_active())
    {
        this->thruster->set(this->altitude_table.values->pressure, time);
        total_force += this->state.CS.axis.z * this->thruster->get_thrust();

        this->update_inertia();
    }

    if(this->parachute->is_deployed())
    {
        this->parachute->update();
        total_force += this->parachute->tether_force;
        total_moment += this->state.CS.axis.z.cross(this->parachute->tether_force) * this->inertia.COG;
    }

    this->state.acceleration = total_force * (1.0/this->inertia.mass);
    this->state.acceleration.z -= this->altitude_table.values->gravity;

    Axis I_inertial = this->state.CS.get_transpose(); // rotate Inertia to inertial frame
    int i = 0;
    for(; i < 6;i++)
    {
        I_inertial.data[i] *= this->inertia.Ixx;
    }
    for(; i < 9;i++)
    {
        I_inertial.data[i] *= this->inertia.Izz;
    }
    this->state.angular_acceleration = I_inertial.get_inverse() * total_moment;
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
        this->inertia.mass -= this->thruster->get_mass_rate()*dt;
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

SingleStageRocket::SingleStageRocket(){}

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
}

