#include "../include/Vehicle.h"

void Vehicle::get_state_rate(const std::array<double,N>& x, const double& t, std::array<double,N>& dx) {
    this->current_stage->set_mass(x[13]);

    this->current_stage->update_force_and_moment();

    dx[13] = this->current_stage->get_mass_rate();

    Vector Force_in_inertial = this->ECI.transpose_mult(this->current_stage.Force);

    double x_inv = 1.0/x[13];
    for(int i = 0; i < 3; i++) {
        dx[i] = x[i3];
        dx[i3] = Force_in_inertial.data[i]*x_inv + this->planet.gravity.acceleration.data[i];
    }

    // Integration of quaternion
    dx[6] = -0.5*(x[10]*x[7] + x[11]*x[8] + x[12]*x[9]);
    dx[7] = 0.5*(x[10]*x[6] + x[12]*x[8] - x[11]*x[9]);
    dx[8] = 0.5*(x[10]*x[7] - x[12]*x[7] + x[10]*x[9]);
    dx[9] = 0.5*(x[10]*x[8] + x[11]*x[7] - x[10]*x[9]);

    // Integration of angular velocity
    if(this->is_symmetric) {
        dx[10] = this->current_stage.Moment.data[0]/this->inertia[0];
        dx[11] = this->current_stage.Moment.data[1]/this->inertia[1];
        dx[12] = this->current_stage.Moment.data[2]/this->inertia[2];
        return;
    }

    if(this->is_plane) {
        dx[11] = (this->current_stage.Moment.data[1] + this->inertia[4]*(x[12]*x[12] - x[10]*x[10]) + x[10]*x[12]*(this->inertia[0] - this->inertia[2]))/this->inertia[1];
        double y1 = this->current_stage.Moment.data[0] + x[11]*(x[12]*(this->inertia[1] - this->inertia[2]) + x[10]*this->inertia[4]);
        double y2 = this->current_stage.Moment.data[2] + x[11]*(x[10]*(this->inertia[0] - this->inertia[1]) - x[12]*this->inertia[4]);

        double det = 1/(this->inertia[0]*this->inertia[2] + this->inertia[4]*this->inertia[4]);

        dx[10] = (this->inertia[2]*y1 - this->inertia[4]*y2)*det;
        dx[12] = (this->inertia[0]*y2 - this->inertia[4]*y1)*det;
        return;
    }

    double Iw[3];
    Iw[0] = this->inertia[0]*x[10] + this->inertia[3]*x[11] + this->inertia[4]*x[12];
    Iw[1] = this->inertia[3]*x[10] + this->inertia[1]*x[11] + this->inertia[5]*x[12];
    Iw[2] = this->inertia[4]*x[10] + this->inertia[5]*x[11] + this->inertia[2]*x[12];

    double wxIw[3];
    Cartesian::cross(&x[10],Iw,wxIx);
    Iw[0] = this->dynamics.Moment.data[0] - wxIw[0];
    Iw[1] = this->dynamics.Moment.data[1] - wxIw[1];
    Iw[2] = this->dynamics.Moment.data[2] - wxIw[2];

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

    dx[10] = (A[0]*Iw[0] + A[1]*Iw[1] + A[2]*Iw[2])*det;
    dx[11] = (A[3]*Iw[0] + A[4]*Iw[1] + A[5]*Iw[2])*det;
    dx[12] = (A[6]*Iw[0] + A[7]*Iw[1] + A[8]*Iw[2])*det;
}
