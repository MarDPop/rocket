#pragma once

#include "../ode/Dynamics.h"
#include "../../lib/Eigen/Dense"
#include <cstring>

/**
 * @brief 
 * 
 */
class Body_Point_Mass : public virtual ODE_Dynamics<7>
{
protected:

    double _time;

    Eigen::Vector3d _position;

    Eigen::Vector3d _velocity;

    Eigen::Vector3d _acceleration;

    double _mass;

    double _mass_rate; 

    inline virtual void set_state_and_time(const std::array<double, 7>& x, double time)
    {
        memcpy(this->_position.data(),&x[0],3*sizeof(double));
        memcpy(this->_velocity.data(),&x[3],3*sizeof(double));
        this->_mass = x[6];
        this->_time = time;
    }

    inline virtual void compute_acceleration(){}

    inline virtual void compute_mass_rate(){} 

    inline virtual bool stop_conditions()
    {
        return true;
    }

public:

    inline bool set_state(const std::array<double, 7>& x, const double& time, std::array<double,7>& dx)
    {
        this->set_state_and_time(x,time);
        this->compute_acceleration();
        this->compute_mass_rate();

        dx[0] = x[3];
        dx[1] = x[4];
        dx[2] = x[5];
        dx[3] = _acceleration[0];
        dx[4] = _acceleration[1];
        dx[5] = _acceleration[2];
        dx[6] = _mass_rate;

        return this->stop_conditions();
    }

};


namespace MOI
{
    union Equal
    {
        double I;
    };

    union Axisymmetric
    {
        double I[2];
        struct 
        {
            double I_axis;
            double I_perp;
        };
    };

    union Principal_Axis
    {
        double I[3];
        struct 
        {
            double I_xx;
            double I_yy;
            double I_zz;
        };
    };

    union XZ_Symmetry
    {
        double I[4];
        struct 
        {
            double I_xx;
            double I_yy;
            double I_zz;
            double I_xz;
        };
    };

    union Full
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

        inline Eigen::Matrix3d get_inertia_matrix() const
        {
            Eigen::Matrix3d inertia;
            double* data = inertia.data();
            data[0] = this->Ixx;
            data[1] = -this->Ixy;
            data[2] = -this->Ixz;
            data[3] = -this->Ixy;
            data[4] = this->Iyy;
            data[5] = -this->Iyz;
            data[6] = -this->Ixz;
            data[7] = -this->Iyz;
            data[8] = this->Izz;
            return inertia;
        }

        inline Full operator+(const Full& moi) const
        {
            Full out;
            for(unsigned i = 0; i < 6; i++)
            {
                out.I[i] = this->I[i] + moi.I[i];
            }
            return out;
        }
    };
};

template<typename T>
struct Inertia
{
    /**
    * current mass  (kg)
    */
    double mass;

    /**
    * current center of mass location from nose (m) [should be negative]
    */
    Eigen::Vector3d CoM;

    /**
    * current principal moment of inertia  (kg m2)
    */
    T MoI;

    inline void operator=(const Inertia<T>& inertia) const
    {
        this->mass = inertia.mass;
        this->CoM = inertia.CoM;
        memcpy(this->MoI.I,inertia.MoI.I,sizeof(MoI));
    }

    inline Inertia<T> operator+(const Inertia<T>& inertia) const;
};

struct Inertia_Full : Inertia<MOI::Full>
{

    inline Inertia<MOI::Full> operator+(const Inertia<MOI::Full>& inertia) const
    {
        Inertia<MOI::Full> out;

        // Get mass first
        out.mass = this->mass + inertia.mass;

        // Compute center of mass
        out.CoM = this->CoM*this->mass + inertia.CoM*inertia.mass;
        out.CoM *= (1.0/out.mass);

        out.MoI = this->MoI + inertia.MoI;

        Eigen::Vector3d r = this->CoM - out.CoM;
        Eigen::Vector3d mr = r*this->mass;
        double mr2 = mr.dot(r);
        out.MoI.Ixx += mr2;
        out.MoI.Iyy += mr2;
        out.MoI.Izz += mr2;

        out.MoI.Ixx -= mr.x()*r.x();
        out.MoI.Iyy -= mr.y()*r.y();
        out.MoI.Izz -= mr.z()*r.z();
        out.MoI.Ixy -= mr.x()*r.y();
        out.MoI.Ixz -= mr.x()*r.z();
        out.MoI.Iyz -= mr.y()*r.z();

        r = inertia.CoM - out.CoM;
        mr = r*inertia.mass;
        mr2 = mr.dot(r);
        out.MoI.Ixx += mr2;
        out.MoI.Iyy += mr2;
        out.MoI.Izz += mr2;

        out.MoI.Ixx -= mr.x()*r.x();
        out.MoI.Iyy -= mr.y()*r.y();
        out.MoI.Izz -= mr.z()*r.z();
        out.MoI.Ixy -= mr.x()*r.y();
        out.MoI.Ixz -= mr.x()*r.z();
        out.MoI.Iyz -= mr.y()*r.z();

        return out;
    }
};


/**
 * @brief 
 * 
 */
template<typename T>
class Body
{
protected:

    double _time;

    Eigen::Vector3d _position;

    Eigen::Vector3d _velocity;

    Eigen::Vector3d _acceleration;

    Eigen::Quaterniond _orientation;

    Eigen::Vector3d _angular_velocity;

    Eigen::Vector3d _angular_acceleration;

    Inertia<T> _inertia;

    Inertia<T> _inertia_rate;

    inline virtual void set_state_and_time(const std::array<double, 7>& x, double time)
    {
        memcpy(this->_position.data(),&x[0],3*sizeof(double));
        memcpy(this->_velocity.data(),&x[3],3*sizeof(double));
        this->_inertia.mass = x[6];
        this->_time = time;
    }

    inline virtual void compute_acceleration(){}

    inline virtual void compute_mass_rate(){} 

    inline virtual bool stop_conditions()
    {
        return true;
    }

public:

    inline bool set_state(const std::array<double, 7>& x, const double& time, std::array<double,7>& dx)
    {
        this->set_state_and_time(x,time);
        this->compute_acceleration();
        this->compute_mass_rate();

        dx[0] = x[3];
        dx[1] = x[4];
        dx[2] = x[5];
        dx[3] = _acceleration[0];
        dx[4] = _acceleration[1];
        dx[5] = _acceleration[2];
        dx[6] = _inertia_rate.mass;

        return this->stop_conditions();
    }

};