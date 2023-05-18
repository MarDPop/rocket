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
#include "Environment.h"
#include "Parachute.h"

using namespace Cartesian;

class SingleStageRocket
{
friend class SingleStageSimulation;

friend class Loader;

    KinematicState state;

    Inertia inertia;

    Inertia inertia_empty;

    MOI MoI_rate_change;

    Axis I_inverse;

    GNC gnc;

    std::unique_ptr<Aerodynamics> aerodynamics;

    std::unique_ptr<Thruster> thruster;

    std::unique_ptr<Parachute> parachute;

    Environment* const _environment;

    bool principal_axis_assumption = false;

    void compute_acceleration(double time);

    /**
    * updates inertia from current mass
    */
    void update_inertia(double inv_dt);

public:

    SingleStageRocket(Environment* atmosphere);
    ~SingleStageRocket();

    inline void set_inertial_properties(Inertia inertia_empty)
    {
        this->inertia_empty = inertia_empty;
        if( (fabs(inertia_empty.MoI.Ixy) + fabs(inertia_empty.MoI.Ixz) + fabs(inertia_empty.MoI.Ixz) < 1e-6) &&
            (fabs(inertia_empty.CoM.x) + fabs(inertia_empty.CoM.y) < 1e-6) )
        {
            this->principal_axis_assumption = true;
            this->I_inverse.zero(); // ensure zero non diagonal terms
        }
    }

    inline const Inertia& get_inertia() const
    {
        return this->inertia;
    }

    inline const Environment& get_environment() const
    {
        return *this->_environment;
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


