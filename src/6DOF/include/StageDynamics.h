#pragma once

#include <memory>

#include "../common/include/Cartesian.h"
#include "../common/include/Dynamics.h"
#include "Aerodynamics.h"
#include "GNC.h"
#include "Thruster.h"

struct Vehicle;

struct StageDynamics : public Dynamics<14> {

    Vehicle* vehicle;

    std::unique_ptr< GNC > gnc;

    std::unique_ptr< Aerodynamics > aero;

    std::unique_ptr< Thruster > thruster;

    Cartesian::Vector Force; // in inertial frame

    std::array<double,3> Moment; // in body frame

    void update_force_and_moment() {

    }

    std::array<double,N> get_state_rate(const std::array<double,N>& x, const double& t) {
        this->update_force_and_moment();


    }

};
