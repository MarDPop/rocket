#pragma once

#include <vector>
#include <memory>
#include <array>
#include <cmath>

#include "../../common/include/Dynamics.h"
#include "Stage.h"
#include "Planet.h"
#include "../../common/include/Cartesian.h"

using namespace Cartesian;


struct State {
    std::array<double,14> x;
    double* const position;
    double* const velocity;
    double* const attitude;
    double* const angular_velocity;
    double* const mass;

    State() : position(&x[0]), velocity(&x[3]), attitude(&x[6]), angular_velocity(&x[10]), mass(&x[13]) { }
};

class Vehicle : public Dynamics<14> {
    int current_stage_idx;

    std::vector< std::unique_ptr< Stage > > stages;

    Stage* current_stage;

public:

    double talo;

    State state;

    std::array<double,6> inertia; // from COG in kg m2

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
        stage->set_vehicle(this);
    }

    void get_state_rate(const std::array<double,14>& x, const double t, std::array<double,14>& dx) override;

};
