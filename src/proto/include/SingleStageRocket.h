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

    void compute_acceleration(double time);

    void step(double& time, double dt);

    void update_inertia();

public:

    Inertia inertia;

    KinematicState state;

    GNC gnc;

    const std::unique_ptr<Aerodynamics> aerodynamics;

    const std::unique_ptr<Thruster> thruster;

    const std::unique_ptr<Parachute> parachute;

    Atmosphere* const atmosphere;

    SingleStageRocket(std::unique_ptr<Aerodynamics> a, std::unique_ptr<Thruster> t, std::unique_ptr<Parachute> p, Atmosphere* atm);
    ~SingleStageRocket();

    static SingleStageRocket load(const char* fn);

    inline void set_inertial_properties(Inertia inertia_empty)
    {
        this->inertia_empty = inertia_empty;
    }

    void init(double launch_angle, double launch_heading);

};


