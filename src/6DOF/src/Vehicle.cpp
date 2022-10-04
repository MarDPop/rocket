#include "../include/Vehicle.h"

#include "../../common/include/util.h"

Vehicle::Vehicle() {}

Vehicle::~Vehicle() {}

void Vehicle::set_orientation(double* q) {
    double mag = 1.0/sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] *= mag;
    q[1] *= mag;
    q[2] *= mag;
    q[3] *= mag;
    util::q2rotm(q,this->inertial_CS.data);
}

void Vehicle::get_state_rate(std::array<double,14>& x, double t, std::array<double,14>& dx) {

    this->set_orientation(&x[6]);

    this->body.update(this,t);

    this->current_stage->set_mass(x[13]);

    this->current_stage->compute();

    dx[13] = this->current_stage->mdot;

    Vector Force_in_inertial = this->inertial_CS.transpose_mult(this->force);

    double x_inv = 1.0/x[13];
    for(int i = 0; i < 3; i++) {
        dx[i] = x[i+3];
        dx[i+3] = Force_in_inertial.data[i]*x_inv + this->body.gravity->acceleration[i];
    }

    // Integration of quaternion
    dx[6] = -0.5*(x[10]*x[7] + x[11]*x[8] + x[12]*x[9]);
    dx[7] = 0.5*(x[10]*x[6] + x[12]*x[8] - x[11]*x[9]);
    dx[8] = 0.5*(x[10]*x[7] - x[12]*x[7] + x[10]*x[9]);
    dx[9] = 0.5*(x[10]*x[8] + x[11]*x[7] - x[10]*x[9]);

    // Integration of angular velocity
    this->compute_moment(&x[10],&dx[10]);
}

void Vehicle::compute_no_moment(double* w, double* dw) {
    dw[0] = 0;
    dw[1] = 0;
    dw[2] = 0;
}

void Vehicle::compute_symmetric_moment(double* w, double* dw) {
    dw[0] = this->moment.data[0]/this->inertia[0];
    dw[1] = this->moment.data[1]/this->inertia[1];
    dw[2] = this->moment.data[2]/this->inertia[2];
}

void Vehicle::compute_plane_moment(double* w, double* dw) {
    dw[1] = (this->moment.data[1] + this->inertia[4]*(w[2]*w[2] - w[0]*w[0]) + w[0]*w[2]*(this->inertia[0] - this->inertia[2]))/this->inertia[1];
    double y1 = this->moment.data[0] + w[1]*(w[2]*(this->inertia[1] - this->inertia[2]) + w[0]*this->inertia[4]);
    double y2 = this->moment.data[2] + w[1]*(w[0]*(this->inertia[0] - this->inertia[1]) - w[2]*this->inertia[4]);

    double det = 1/(this->inertia[0]*this->inertia[2] + this->inertia[4]*this->inertia[4]);

    dw[0] = (this->inertia[2]*y1 - this->inertia[4]*y2)*det;
    dw[2] = (this->inertia[0]*y2 - this->inertia[4]*y1)*det;
}

void Vehicle::compute_full_moment(double* w, double* dw){
    double Iw[3];
    Iw[0] = this->inertia[0]*w[0] + this->inertia[3]*w[1] + this->inertia[4]*w[2];
    Iw[1] = this->inertia[3]*w[0] + this->inertia[1]*w[1] + this->inertia[5]*w[2];
    Iw[2] = this->inertia[4]*w[0] + this->inertia[5]*w[1] + this->inertia[2]*w[2];

    double wxIw[3];
    Cartesian::cross(w,Iw,wxIw); // NOTE: Due to this call, x must be the this->state.x
    Iw[0] = this->moment.data[0] - wxIw[0];
    Iw[1] = this->moment.data[1] - wxIw[1];
    Iw[2] = this->moment.data[2] - wxIw[2];

    double A[9];
    A[0] = this->inertia[1]*this->inertia[2] - this->inertia[5]*this->inertia[5];
    A[1] = this->inertia[4]*this->inertia[5] - this->inertia[3]*this->inertia[2];
    A[2] = this->inertia[3]*this->inertia[5] - this->inertia[4]*this->inertia[1];

    A[3] = this->inertia[5]*this->inertia[4] - this->inertia[3]*this->inertia[2];
    A[4] = this->inertia[0]*this->inertia[2] - this->inertia[4]*this->inertia[4];
    A[5] = this->inertia[3]*this->inertia[3] - this->inertia[0]*this->inertia[5];

    A[6] = this->inertia[3]*this->inertia[5] - this->inertia[1]*this->inertia[4];
    A[7] = this->inertia[3]*this->inertia[4] - this->inertia[0]*this->inertia[5];
    A[8] = this->inertia[0]*this->inertia[1] - this->inertia[3]*this->inertia[3];

    double det = 1.0/(this->inertia[0]*A[0] + this->inertia[3]*A[3] + this->inertia[4]*A[6]);

    dw[0] = (A[0]*Iw[0] + A[1]*Iw[1] + A[2]*Iw[2])*det;
    dw[1] = (A[3]*Iw[0] + A[4]*Iw[1] + A[5]*Iw[2])*det;
    dw[2] = (A[6]*Iw[0] + A[7]*Iw[1] + A[8]*Iw[2])*det;
}

void SingleStageRocket::compute_acceleration() {

    if(this->mass > this->mass_empty) {
        this->thruster.set(this->pressure);
        this->acceleration = this->CS.axis.z * this->thruster.thrust;
    }

    this->aero.update();

    this->acceleration += this->aero.force;

    this->torque = this->aero.moment;

    this->acceleration.data[2] -= this->grav;

}

void SingleStageRocket::set_mass(double empty_mass, double full_mass, double Izz_ratio, double Ixx_ratio) {
    this->mass_empty = empty_mass;
    this->mass_full = full_mass;
    this->Izz_ratio = Izz_ratio;
    this->Ixx_ratio = Ixx_ratio;
}

void SingleStageRocket::set_ground(double ground_altitude,double ground_pressure,double ground_temperature, double lapse_rate) {
    this->ground_altitude = ground_altitude;
    this->ground_pressure = ground_pressure;
    this->ground_temperature = ground_temperature;
    this->lapse_rate = lapse_rate;
}

void SingleStageRocket::launch(double dt) {
    double time = 0;
    double time_record = 0;
    double dt_half = dt*0.5;

    double g;
    while(time < 10000) {

        this->compute_acceleration();

        Vector p0 = this->position;
        Vector v0 = this->velocity;
        Vector a0 = this->acceleration;

        this->position += this->velocity*dt;
        this->velocity += this->acceleration*dt;

        Vector rotation = this->angular_velocity*dt;
        double angle = rotation.norm();
        rotation *= (1.0/angle);
        Axis rot_mat;
        Cartesian::rotation_matrix_angle_axis(angle,rotation,rot_mat);



        this->angular_velocity += this->angular_acceleration*dt;

        if(this->mass > this->mass_empty) {
            this->mass -= this->thruster.mass_rate*dt;
        }

        this->compute_acceleration();

        this->position = p0 + (v0 + this->velocity)*dt_half;
        this->velocity = v0 + (a0 + this->acceleration)*dt_half;

        if(time > time_record){
            time_record += this->record.t_interval;

            if(this->position.z() < -0.5) {
                break;
            }

            double r = (this->position.z() + 6371000 + this->ground_altitude)*1.56961e-7;
            this->grav = 9.806*r*r;

            this->sound_speed_inv = 1.0/sqrt(air_const*(this->ground_temperature + this->lapse_rate*this->position.z()));
        }

        time += dt;
    }
}
