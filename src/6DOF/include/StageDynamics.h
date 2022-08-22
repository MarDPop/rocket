#pragma once

#include <memory>

#include "../../common/include/Cartesian.h"
#include "../../common/include/Dynamics.h"
#include "Aerodynamics.h"
#include "GNC.h"
#include "Thruster.h"

struct Vehicle;

struct StageDynamics
{

    Vehicle* vehicle = nullptr;

public:

    std::unique_ptr< GNC > gnc;

    std::unique_ptr< Aerodynamics > aero;

    std::unique_ptr< Thruster > thruster;

    std::vector< std::unique_ptr< Action > > other_forces;

    Cartesian::Vector Force; // in inertial frame at COG

    Cartesian::Vector Moment; // in body frame at COG

    double mdot;

    void set_vehicle(Vehicle* vehicle)
    {
        this->vehicle = vehicle;
    }

    Vehicle* get_vehicle()
    {
        return this->vehicle;
    }

    void update_force_and_moment(const double& t);

};
