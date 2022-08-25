#pragma once

#include <array>
#include <memory>
#include "Action.h"
#include "GNC.h"
#include "Aerodynamics.h"
#include "Thruster.h"

class Vehicle;

class Stage {

friend class Vehicle;

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

    double mdot;

    Vehicle* vehicle;

public:

    std::unique_ptr< GNC > gnc;

    std::unique_ptr< Aerodynamics > aero;

    std::unique_ptr< Thruster > thruster;

    std::vector< std::unique_ptr< Action > > actions;

    Stage(const double& empty, const double& full, const std::array<double,6>& empty_i, const std::array<double,6>& full_i,
            const std::array<double,3>& empty_x, const std::array<double,3>& full_x);

    void set_mass(double mass);

    inline void set_vehicle(Vehicle* v) {
        this->vehicle = v;
    }

    void compute();

};
