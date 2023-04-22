#pragma once

#include "../../common/include/Cartesian.h"

#include "string.h"

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
    Axis CS; // TODO: make this quaternion

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

union MOI
{
    double I[6];
    struct {
        double Ixx;
        double Iyy;
        double Izz;
        double Ixy;
        double Ixz;
        double Iyz;
    };

    inline Axis get_inertia_matrix() const
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

    inline MOI operator+(const MOI& moi) const
    {
        MOI out;
        for(unsigned i = 0; i < 6; i++)
        {
            out.I[i] = this->I[i] + moi.I[i];
        }
        return out;
    }

    inline Vector operator*(const Vector& vec) const
    {
        Vector out;

        out.x = this->Ixx*vec.x;
        out.y = this->Iyy*vec.y;
        out.z = this->Izz*vec.z;

        return out;
    }
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
    MOI MoI;

    /**
    * current center of mass location from nose (m) [should be negative]
    */
    Vector CoM;

    inline void set_from_basic(const Inertia_Basic& basic)
    {
        std::fill_n(this->MoI.I,6,0.0);
        this->CoM.zero();

        this->mass = basic.mass;

        this->MoI.Ixx = basic.Ixx;
        this->MoI.Iyy = basic.Ixx;
        this->MoI.Izz = basic.Izz;

        this->CoM.z = basic.CoM_axial;
    }

    inline void operator=(const Inertia& inertia)
    {
        this->mass = inertia.mass;
        std::copy_n(inertia.MoI.I,6,this->MoI.I);
        this->CoM = inertia.CoM;
    }

    inline Inertia operator+(const Inertia& inertia) const
    {
        Inertia out;

        // Get mass first
        out.mass = this->mass + inertia.mass;

        // Compute center of mass
        out.CoM = this->CoM*this->mass + inertia.CoM*inertia.mass;
        out.CoM *= (1.0/out.mass);

        out.MoI = this->MoI + inertia.MoI;

        Vector r = this->CoM - out.CoM;
        Vector mr = r*this->mass;
        double mr2 = mr.dot(r);
        out.MoI.Ixx += mr2;
        out.MoI.Iyy += mr2;
        out.MoI.Izz += mr2;

        out.MoI.Ixx -= mr.x*r.x;
        out.MoI.Iyy -= mr.y*r.y;
        out.MoI.Izz -= mr.z*r.z;
        out.MoI.Ixy -= mr.x*r.y;
        out.MoI.Ixz -= mr.x*r.z;
        out.MoI.Iyz -= mr.y*r.z;

        r = inertia.CoM - out.CoM;
        mr = r*inertia.mass;
        mr2 = mr.dot(r);
        out.MoI.Ixx += mr2;
        out.MoI.Iyy += mr2;
        out.MoI.Izz += mr2;

        out.MoI.Ixx -= mr.x*r.x;
        out.MoI.Iyy -= mr.y*r.y;
        out.MoI.Izz -= mr.z*r.z;
        out.MoI.Ixy -= mr.x*r.y;
        out.MoI.Ixz -= mr.x*r.z;
        out.MoI.Iyz -= mr.y*r.z;

        return out;
    }
};
