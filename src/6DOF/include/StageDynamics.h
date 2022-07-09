#pragma once

#include <memory>

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

    std::array<double,3> Force;

    std::array<double,3> Moment;

    

};