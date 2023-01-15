#include "../include/SingleStageRocket.h"

#include "../../common/include/util.h"
#include <fstream>
#include <numeric>
#include <iostream>

SingleStageRocket::SingleStageRocket() : aerodynamics(*this) {

}
SingleStageRocket::~SingleStageRocket(){}

void SingleStageRocket::init(double launch_angle, double launch_heading) {
    if(launch_angle == 0) {
        this->CS.identity();
    } else {
        double cphi = cos(launch_heading);
        double sphi = sin(launch_heading);
        double ctheta = cos(launch_angle);
        double stheta = sqrt(1 - ctheta*ctheta);
        this->CS.axis.x.x = cphi;
        this->CS.axis.x.y = -sphi;
        this->CS.axis.x.z = 0;
        this->CS.axis.y.x = ctheta*sphi;
        this->CS.axis.y.y = ctheta*cphi;
        this->CS.axis.y.z = -stheta;
        this->CS.axis.z.x = stheta*sphi;
        this->CS.axis.z.y = stheta*cphi;
        this->CS.axis.z.z = ctheta;
        //Cartesian::cross(this->CS.axis.z.data,this->CS.axis.x.data,this->CS.axis.y.data);
    }
    this->position.zero();
    this->velocity.zero();
    this->angular_velocity.zero();
    this->mass = this->mass_full;
    this->Ixx = this->I_empty[1] + this->dIdm[1]*(this->mass - this->mass_empty);
    this->Izz = this->I_empty[1] + this->dIdm[1]*(this->mass - this->mass_empty);
}

void SingleStageRocket::compute_acceleration(double time) {

    this->altitude_table.set(this->position[2], time);

    this->aerodynamics.update();

    this->acceleration = this->aerodynamics.force;
    this->angular_acceleration = this->aerodynamics.moment;

    if(this->mass > this->mass_empty) {
        this->thruster.set(this->altitude_table.values->pressure);
        this->acceleration += this->CS.axis.z * this->thruster.thrust;
    }

    if(this->control){
        this->control->update(time);

        this->acceleration += this->control->dForce;
        this->angular_acceleration += this->control->dMoment;
    }

    this->acceleration *= (1.0/this->mass);
    this->acceleration.data[2] -= this->altitude_table.values->gravity;

    Axis I_inertial = this->CS.get_transpose(); // might have to transpose CS
    int i = 0;
    for(; i < 6;i++) {
        I_inertial.data[i] *= this->Ixx;
    }
    for(; i < 9;i++) {
        I_inertial.data[i] *= this->Izz;
    }
    //Vector Iw = I_inertial * this->angular_velocity;
    //Vector rhs = this->angular_acceleration - this->angular_velocity.cross(Iw);
    // this->angular_acceleration = I_inertial.get_inverse() * rhs;
    this->angular_acceleration = I_inertial.get_inverse() * this->angular_acceleration;
}

void SingleStageRocket::set_mass(double empty_mass, double full_mass, double I_empty[3], double I_full[3]) {
    this->mass_empty = empty_mass;
    this->mass_full = full_mass;
    this->I_empty[0] = I_empty[0]; // Ixx
    this->I_empty[1] = I_empty[1]; // Izz
    this->I_empty[2] = I_empty[2]; // COG
    double dm = full_mass - empty_mass;
    this->dIdm[0] = (I_full[0] - I_empty[0])/dm;
    this->dIdm[1] = (I_full[1] - I_empty[1])/dm;
    this->dIdm[2] = (I_full[2] - I_empty[2])/dm; // dCOG
    this->Ixx = I_full[0];
    this->Izz = I_full[1];
    this->COG = I_full[2];
}

void SingleStageRocket::get_inertia() {
    double dm = this->mass - this->mass_empty;
    this->Ixx = this->I_empty[0] + this->dIdm[0]*dm;
    this->Izz = this->I_empty[1] + this->dIdm[1]*dm;
    this->COG = this->I_empty[2] + this->dIdm[2]*dm;
}


void SingleStageRocket::launch(double dt) {
    double time = 0;
    double time_record = 0;
    double dt_half = dt*0.5;

    bool burnout = false;

    Axis mat,M0;
    Vector p0,v0,a0,w0,t0;
    while(time < 10000) {

        // Get initial state
        this->compute_acceleration(time);

        p0 = this->position;
        v0 = this->velocity;
        a0 = this->acceleration;
        w0 = this->angular_velocity;
        t0 = this->angular_acceleration;
        M0 = this->CS;

        // propagate to time + dt

        this->position += this->velocity*dt;
        this->velocity += this->acceleration*dt;

        Vector w = this->angular_velocity*dt;
        double angle = w.norm();
        if(angle > 1e-6) {
            w *= (1.0/angle);
            Cartesian::rotation_matrix_angle_axis(angle,w,mat);
            this->CS = mat*M0;
        }

        this->angular_velocity += this->angular_acceleration*dt;

        if(this->mass > this->mass_empty) {
            this->mass -= this->thruster.mass_rate*dt;
            this->get_inertia();
        }

        // recompute state rate at time + dt
        this->compute_acceleration(time + dt);

        this->position = p0 + (v0 + this->velocity)*dt_half;
        this->velocity = v0 + (a0 + this->acceleration)*dt_half;

        w = (w0 + this->angular_velocity)*dt_half;
        angle = w.norm();
        if(angle > 1e-6) {
            w *= (1.0/angle);
            Cartesian::rotation_matrix_angle_axis(angle,w,mat);
            this->CS = mat*M0;
        }

        this->angular_velocity = w0 + (t0 + this->angular_acceleration)*dt_half;

        if(time > time_record){
            this->record.position.push_back(this->position);
            this->record.orientation.push_back(this->CS);
            this->record.mass.push_back(this->mass);

            if(std::isnan(this->position.data[0])){
                break;
            }

            time_record += this->record.t_interval;

            if(this->position.z < -0.5) {
                break;
            }

            if(!burnout && this->mass <= this->mass_empty) {
                burnout = true;
            }
        }

        time += dt;
    }
}
