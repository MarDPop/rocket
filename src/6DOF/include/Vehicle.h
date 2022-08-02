#pragma once

#include <vector>
#include <memory>
#include <array>
#include <cmath>

#include "StageDynamics.h"
#include "Planet.h"
#include "../../common/include/Cartesian.h"

using namespace Cartesian;

class Stage : public Dynamics<14> {
    const double mass_empty;
    const double mass_full;
    const std::array<double,6> inertia_empty;
    const std::array<double,6> inertia_full;
    const std::array<double,3> COG_empty;
    const std::array<double,3> COG_full;

    double dm;
    std::array<double,6> dIdm;
    std::array<double,3> dCGdm;

    bool is_symmetric;
    bool is_plane;
    bool is_ballistic;
    bool is_3DOF;
public:
    StageDynamics dynamics;

    double mass; // in kg
    std::array<double,6> inertia; // from COG in kg m2
    std::array<double,3> COG; // from nose (reference) in m

    Stage(const double& empty, const double& full, const std::array<double,6>& empty_i, const std::array<double,6>& full_i,const std::array<double,3>& empty_x, const std::array<double,3>& full_x) :
    mass_empty(empty), mass_full(full), inertia_empty(empty_i), inertia_full(full_i), COG_empty(empty_x), COG_full(full_x) {
        this->dm = full - empty;

        if(this->dm < 1e-3){
            this->is_ballistic = true;
            return;
        }

        this->is_ballistic = false;
        for(int i = 0; i < 3; i++) {
            this->dCGdm[i] = (full_COG[i] - empty_COG[i])/dm;
            this->dIdm[i] = (full_inertia[i] - empty_inertia[i])/dm;
        }

        for(int i = 3; i < 6; i++) {
            this->dIdm[i] = (full_inertia[i] - empty_inertia[i])/dm;
        }

        if(fabs(this->full_inertia[3]) < 1e-6 && fabs(this->full_inertia[4]) < 1e-6) {
            this->is_plane = true;
        } else {
            this->is_plane = false;
        }

        if(this->is_plane && fabs(this->full_inertia[4]) < 1e-6) {
            this->is_symmetric = true;
        } else {
            this->is_symmetric = false;
        }
    }

    void set_mass(const double& mass) {
        this->mass = mass;

        double dm = mass - this->mass_empty;

        int i = 0;
        while(i < 3) {
            this->COG[i] = this->COG_empty[i] + this->dCGdm[i]*dm;
            this->inertia[i] = this->inertia_empty[i] + this->dIdm[i]*dm;
            i++;
        }

        if(this->is_symmetric) {
            return;
        }

        while(i < 6) {
            this->inertia[i] = this->inertia_empty[i] + this->dIdm[i]*dm;
            i++;
        }

    }

    void get_state_rate(const std::array<double,N>& x, const double& t,std::array<double,N>& dx) {
        this->set_mass(x[13]);

        this->dynamics.update_force_and_moment();

        dx[13] = this->dynamics->mdot;

        Vector Force_in_inertial = this->vehicle->ECI.transpose_mult(this->dynamics.Force);

        double x_inv = 1.0/x[13];
        for(int i = 0; i < 3; i++) {
            dx[i] = x[i3];
            dx[i3] = Force_in_inertial.data[i]*x_inv;
        }

        // Integration of quaternion
        dx[6] = -0.5*(x[10]*x[7] + x[11]*x[8] + x[12]*x[9]);
        dx[7] = 0.5*(x[10]*x[6] + x[12]*x[8] - x[11]*x[9]);
        dx[8] = 0.5*(x[10]*x[7] - x[12]*x[7] + x[10]*x[9]);
        dx[9] = 0.5*(x[10]*x[8] + x[11]*x[7] - x[10]*x[9]);

        // Integration of angular velocity
        if(this->is_symmetric) {
            dx[10] = this->dynamics.Moment.data[0]/this->inertia[0];
            dx[11] = this->dynamics.Moment.data[1]/this->inertia[1];
            dx[12] = this->dynamics.Moment.data[2]/this->inertia[2];
            return;
        }

        if(this->is_plane) {
            dx[11] = (this->dynamics.Moment.data[1] + this->inertia[5]*(x[12]*x[12] - x[10]*x[10]) + x[10]*x[12]*(this->inertia[0] - this->inertia[2]))/this->inertia[1];
            double y1 = this->dynamics.Moment.data[0] + x[11]*(x[12]*(this->inertia[1] - this->inertia[2]) + x[10]*this->inertia[5]);
            double y2 = this->dynamics.Moment.data[2] + x[11]*(x[10]*(this->inertia[0] - this->inertia[1]) - x[12]*this->inertia[5]);

            double det = 1/(this->inertia[0]*this->inertia[2] + this->inertia[5]*this->inertia[5]);

            dx[10] = (this->inertia[2]*y1 - this->inertia[5]*y2)*det;
            dx[12] = (this->inertia[0]*y2 - this->inertia[5]*y1)*det;
        }

    }
};

struct State {
    std::array<double,14> x;
    double* const position;
    double* const velocity;
    double* const attitude;
    double* const angular_velocity;
    double* const mass;

    State() : position(&x[0]), velocity(&x[3]), attitude(&x[6]), angular_velocity(&x[10]), mass(&x[13]) { }
};

class Vehicle {
    int current_stage_idx;

    std::vector< std::unique_ptr< Stage > > stages;

    Stage* current_stage;

public:

    double talo;

    State state;

    State dstate;

    Axis ECI;

    Planet planet;

    Axis ENU;

    std::array< double, 3 > LLA;

    Vector body_fixed_pos;

    Vector body_fixed_velocity;

    Vehicle();
    ~Vehicle();

    inline void add_stage(Stage* stage) {
        stages.emplace_back(stage);
    }

};
