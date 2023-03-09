#pragma once

#include <vector>
#include <memory>
#include <string>

#include "../../common/include/Cartesian.h"

#include "Aerodynamics.h"
#include "Control.h"
#include "Thruster.h"
#include "Atmosphere.h"
#include "Parachute.h"

using namespace Cartesian;

struct KinematicState
{
    /**
    * Current position in ENU (m)
    */
    Vector position;

    /**
    * Current velocity in ENU (m/s)
    */
    Vector velocity;

    /**
    * Current position in ENU (m/s2)
    */
    Vector acceleration;

    /**
    * Current body frame in ENU (z axis is axial)
    */
    Axis CS;

    /**
    * Current angular speed (rad/s)
    */
    Vector angular_velocity;

    /**
    * Current angular acceleration (rad/s)
    */
    Vector angular_acceleration;
};

struct Inertia
{
    /**
    * current mass  (kg)
    */
    double mass;

    /**
    * current principal moment of inertia cross axially (kg m2)
    */
    double Ixx;

    /**
    * current principal moment of inertia along axis  (kg m2)
    */
    double Izz;

    /**
    * current center of mass location from nose (m) [should be negative]
    */
    double COG;
};

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

    std::unique_ptr<Control> control;

    std::unique_ptr<Parachute> parachute;

    SingleStageRocket();
    ~SingleStageRocket();

    void set_inertial_properties(double empty_mass, double full_mass, double I_empty[3], double I_full[3]);

    void init(double launch_angle, double launch_heading);

};


