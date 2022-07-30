#pragma once

#include <vector>
#include <memory>
#include <array>
#include <cmath>

#include "StageDynamics.h"
#include "Planet.h"

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

        this->is_symmetric = true;
        for(int i = 3; i < 6; i++) {
            this->dIdm[i] = (full_inertia[i] - empty_inertia[i])/dm;
            if(fabs(this->full_inertia[i]) > 1e-6) {
                this->is_symmetric = false;
            }
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
        dynamics.update_force_and_moment();

        for(int i = 0; i < 3; i++) {

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

    Planet planet;

    std::array< std::array< double, 3 >, 3 > ECI_Axis;

    std::array< double, 3 > LLA;

    std::array< double, 3 > body_fixed_pos;

    std::array< double, 3 > body_fixed_velocity;

    Vehicle();
    ~Vehicle();

    inline void add_stage(Stage* stage) {
        stages.emplace_back(stage);
    }

};
