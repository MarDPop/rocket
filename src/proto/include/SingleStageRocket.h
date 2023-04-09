#pragma once

#include <vector>
#include <array>
#include <memory>
#include <string>

#include "../../common/include/Cartesian.h"

#include "Kinematics.h"

#include "Aerodynamics.h"
#include "GNC.h"
#include "Thruster.h"
#include "Atmosphere.h"
#include "Parachute.h"

using namespace Cartesian;

class SingleStageRocket
{
friend class SingleStageSimulation;

    Inertia inertia_empty;

    Inertia inertia;

    GNC gnc;

    std::unique_ptr<Aerodynamics> aerodynamics;

    std::unique_ptr<Thruster> thruster;

    std::unique_ptr<Parachute> parachute;

    Atmosphere* atmosphere;

    void compute_acceleration(double time);

    void step(double& time, double dt);

    void update_inertia();

public:

    KinematicState state;

    SingleStageRocket();
    ~SingleStageRocket();

    void load(const char* fn);

    inline void set_inertial_properties(Inertia inertia_empty)
    {
        this->inertia_empty = inertia_empty;
    }

    inline void set_atmosphere(Atmosphere* atmosphere)
    {
        this->atmosphere = atmosphere;
    }

    void init(double launch_angle, double launch_heading);

};


