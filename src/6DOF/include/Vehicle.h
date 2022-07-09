#pragma once

#include <vector>
#include <memory>
#include <array>

#include "StageDynamics.h"
#include "Planet.h"

class Stage {
    const double empty_mass;
    const double full_mass;
    const std::array<double,6> empty_inertia;
    const std::array<double,6> full_inertia;
    const std::array<double,3> empty_COG;
    const std::array<double,3> full_COG;

    double dm;
    std::array<double,6> dIdm;
    std::array<double,3> dCGdm;

    bool is_symmetric;
    bool is_ballistic;
public:
    std::unique_ptr< StageDynamics > dynamics;

    double mass; // in kg
    std::array<double,6> inertia; // from COG in kg m2
    std::array<double,3> COG; // from nose (reference) in m

    Stage(const double& empty, const double& full, const std::array<double,6>& empty_i, const std::array<double,6>& full_i,const std::array<double,3>& empty_x, const std::array<double,3>& full_x) :
    empty_mass(empty), full_mass(full), empty_inertia(empty_i), full_inertia(full_i), empty_COG(empty_x), full_COG(full_x) {
        this->dm = full - empty;

        if(this->dm < 1e-3){
            this->is_ballistic = false;
            return;
        }

        this->is_ballistic = true;
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
    Stage* current_stage;

    std::vector< std::unique_ptr< Stage > > stages;

public:

    double talo;

    State state;

    State dstate;

    Planet planet;

    std::array< double, 3 > LLA;

    std::array< double, 3 > body_fixed_pos;

    std::array< double, 3 > body_fixed_velocity;

    Vehicle();
    ~Vehicle();

};