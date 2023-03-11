#pragma once

#include <vector>
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

    bool not_empty = false;

    /**
    * Mass dry (kg)
    */
    double mass_empty;

    /**
    * Mass with propellant (kg)
    */
    double mass_full;

    /**
    * Inertia properties : Ixx, Izz, COG
    */
    double I_empty[3];

    /**
    * change in Inertia vs mass
    */
    double dIdm[3];

    /**
    *
    */
    void compute_acceleration(double time);

    void step(double& time, double dt);

    void get_inertia();

public:

    Inertia inertia;

    KinematicState state;

    std::unique_ptr<Atmosphere> atmosphere;

    std::unique_ptr<Aerodynamics> aerodynamics;

    std::unique_ptr<Thruster> thruster;

    std::unique_ptr<GNC> gnc;

    std::unique_ptr<Parachute> parachute;

    SingleStageRocket();
    ~SingleStageRocket();

    void set_inertial_properties(double empty_mass, double full_mass, double I_empty[3], double I_full[3]);

    void init(double launch_angle, double launch_heading);

};


