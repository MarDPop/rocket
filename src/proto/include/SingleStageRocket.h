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

    KinematicState state;

    Inertia inertia_empty;

    Inertia inertia;

    GNC gnc;

    std::unique_ptr<Aerodynamics> aerodynamics;

    std::unique_ptr<Thruster> thruster;

    std::unique_ptr<Parachute> parachute;

    Atmosphere* const _atmosphere;

    void compute_acceleration(double time);

    void step(double& time, double dt);

    void update_inertia();

public:

    SingleStageRocket(Atmosphere* atmosphere);
    ~SingleStageRocket();

    void load(const char* fn);

    inline void set_inertial_properties(Inertia inertia_empty)
    {
        this->inertia_empty = inertia_empty;
    }

    inline const Inertia& get_inertia() const
    {
        return this->inertia;
    }

    inline const Atmosphere& get_atmosphere() const
    {
        return *this->_atmosphere;
    }

    inline const KinematicState& get_state() const
    {
        return this->state;
    }

    inline const Aerodynamics& get_aerodynamics() const
    {
        return *this->aerodynamics;
    }

    inline const Thruster& get_thruster() const
    {
        return *this->thruster;
    }

    void init(double launch_angle, double launch_heading);

};


