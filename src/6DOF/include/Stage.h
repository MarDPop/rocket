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

    Vehicle* vehicle = nullptr;

    std::vector< std::unique_ptr< Action > > actions;

    GNC* gnc = nullptr;
    Aerodynamics* aero = nullptr;
    Thruster* thruster = nullptr;

public:

    Stage(const double& empty_m, const double& full_m, const std::array<double,6>& empty_i, const std::array<double,6>& full_i,
            const std::array<double,3>& empty_x, const std::array<double,3>& full_x);

    void set_mass(double mass);

    inline void set_vehicle(Vehicle* v) {
        this->vehicle = v;
    }

    inline void set_GNC(std::unique_ptr<GNC> gnc) {
        this->gnc = gnc.get();
        this->actions[0] = std::move(gnc);
    }

    inline void set_aerodynamics(std::unique_ptr<Aerodynamics> aero) {
        this->aero = aero.get();
        this->actions[1] = std::move(aero);

    }

    inline void set_thruster(std::unique_ptr<Thruster> thruster) {
        this->thruster = thruster.get();
        this->actions[2] = std::move(thruster);
    }

    void compute();

};
