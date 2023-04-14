#pragma once

#include "../../common/include/Cartesian.h"

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
    * Current angular speed in body frame (rad/s)
    */
    Vector angular_velocity;

    /**
    * Current angular acceleration in body frame (rad/s)
    */
    Vector angular_acceleration;
};

struct Inertia_Basic
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
    double CoM_axial;
};

struct Inertia
{
    /**
    * current mass  (kg)
    */
    double mass;

    /**
    * current principal moment of inertia  (kg m2)
    */
    union
    {
        double I[6];
        struct {
            double Ixx;
            double Iyy;
            double Izz;
            double Ixy;
            double Ixz;
            double Izy;
        };
    } MOI;

    /**
    * current center of mass location from nose (m) [should be negative]
    */
    Vector CoM;

    inline Axis get_inertia_matrix()
    {
        Axis inertia;
        inertia.data[0] = this->Ixx;
        inertia.data[1] = -this->Ixy;
        inertia.data[2] = -this->Ixz;
        inertia.data[3] = -this->Ixy;
        inertia.data[4] = this->Iyy;
        inertia.data[5] = -this->Iyz;
        inertia.data[6] = -this->Ixz;
        inertia.data[7] = -this->Iyz;
        inertia.data[8] = this->Izz;
        return inertia;
    }
};
