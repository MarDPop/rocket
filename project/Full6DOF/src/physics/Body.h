#pragma once

#include "../ode/Dynamics.h"
#include "../../lib/Eigen/Dense"
#include <cstring>

/**
 * @brief 
 * 
 */
class Body_Point_Mass : public virtual Fixed_Size_Dynamics<7>
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

/**
 * @brief 
 * 
 */
/*
Note on constants:
    - for Plane Symmetry the plane is assumed to be in the XZ axis
    - for Axisymmetric the axis is assumed to be on the Z axis
*/
enum MOMENT_CONSTANTS {EQUAL = 1u, AXISYMMETRIC = 2u, PRINCIPAL_AXIS = 3u, PLANE_SYMMETRY = 4u, FULL = 6u};


template<MOMENT_CONSTANTS NDEG>
struct MomentOfInertia
{
    std::array<double, NDEG> I; // Order is Ixx, Iyy, Izz, Ixy, Ixz, Iyz

    inline MomentOfInertia operator+(const MomentOfInertia& moi) const
    {
        MomentOfInertia output;
        for(unsigned idx = 0; idx < NDEG; idx++)
        {
            output.I[idx] = I[idx] + moi[idx];
        }
        return output;
    }

    inline Eigen::Matrix3d get_inertia_matrix() const;
};

inline Eigen::Matrix3d MomentOfInertia<FULL>::get_inertia_matrix() const
{
    Eigen::Matrix3d inertia;
    double* data = inertia.data();
    data[0] = this->I[0];
    data[4] = this->I[1];
    data[8] = this->I[2];
    data[1] = data[3] = -this->I[3];
    data[2] = data[6] = -this->I[4];
    data[5] = data[7] = -this->I[5];
    return inertia;
}

inline Eigen::Matrix3d MomentOfInertia<PLANE_SYMMETRY>::get_inertia_matrix() const
{
    Eigen::Matrix3d inertia;
    double* data = inertia.data();
    data[0] = this->I[0];
    data[4] = this->I[1];
    data[8] = this->I[2];
    data[1] = data[3] = 0.0;
    data[2] = data[6] = -this->I[3];
    data[5] = data[7] = 0.0;
    return inertia;
}

inline Eigen::Matrix3d MomentOfInertia<PRINCIPAL_AXIS>::get_inertia_matrix() const
{
    Eigen::Matrix3d inertia;
    double* data = inertia.data();
    data[0] = this->I[0];
    data[4] = this->I[1];
    data[8] = this->I[2];
    data[1] = data[2] = data[3] = 0.0;
    data[5] = data[6] = data[7] = 0.0;
    return inertia;
}

inline Eigen::Matrix3d MomentOfInertia<AXISYMMETRIC>::get_inertia_matrix() const
{
    Eigen::Matrix3d inertia;
    double* data = inertia.data();
    data[0] = data[4] = this->I[0];
    data[8] = this->I[1];
    data[1] = data[2] = data[3] = 0.0;
    data[5] = data[6] = data[7] = 0.0;
    return inertia;
}

inline Eigen::Matrix3d MomentOfInertia<EQUAL>::get_inertia_matrix() const
{
    Eigen::Matrix3d inertia;
    double* data = inertia.data();
    data[0] = data[4] = data[8] = this->I[0];
    data[1] = data[2] = data[3] = 0.0;
    data[5] = data[6] = data[7] = 0.0;
    return inertia;
}

template<MOMENT_CONSTANTS NDEG>
struct Inertia
{
    /**
    * current mass  (kg)
    */
    double mass;

    /**
    * current center of mass location from nose (m) [should be negative]
    */
    Eigen::Vector3d center_of_mass;

    /**
    * current principal moment of inertia  (kg m2)
    */
    MomentOfInertia<NDEG> moment_of_inertia;

    inline void operator=(const Inertia<NDEG>& inertia) const
    {
        this->mass = inertia.mass;
        this->center_of_mass = inertia.center_of_mass;
        memcpy(this->moment_of_inertia.I.data(),inertia.moment_of_inertia.I.data(),NDEG*sizeof(double));
    }

    inline Inertia<FULL> operator+(const Inertia<NDEG>& inertia) const
    {
        Inertia<FULL> output;

        // Get mass first
        output.mass = this->mass + inertia.mass;

        // Compute center of mass
        output.center_of_mass = this->center_of_mass*this->mass + inertia.center_of_mass*inertia.mass;
        output.center_of_mass *= (1.0/out.mass);

        output.moment_of_inertia = this->moment_of_inertia + inertia.moment_of_inertia;

        Eigen::Vector3d r = this->center_of_mass - output.center_of_mass;
        Eigen::Vector3d mr = r*this->mass;
        double mr2 = mr.dot(r);
        output.moment_of_inertia.I[0] += mr2;
        output.moment_of_inertia.I[1] += mr2;
        output.moment_of_inertia.I[2] += mr2;

        output.moment_of_inertia.I[0] -= mr.x()*r.x();
        output.moment_of_inertia.I[1] -= mr.y()*r.y();
        output.moment_of_inertia.I[2] -= mr.z()*r.z();
        output.moment_of_inertia.I[3] -= mr.x()*r.y();
        output.moment_of_inertia.I[4] -= mr.x()*r.z();
        output.moment_of_inertia.I[5] -= mr.y()*r.z();

        r = input.center_of_mass - output.center_of_mass;
        mr = r*input.mass;
        mr2 = mr.dot(r);
        output.moment_of_inertia.I[0] += mr2;
        output.moment_of_inertia.I[1] += mr2;
        output.moment_of_inertia.I[2] += mr2;

        output.moment_of_inertia.I[0] -= mr.x()*r.x();
        output.moment_of_inertia.I[1] -= mr.y()*r.y();
        output.moment_of_inertia.I[2] -= mr.z()*r.z();
        output.moment_of_inertia.I[3] -= mr.x()*r.y();
        output.moment_of_inertia.I[4] -= mr.x()*r.z();
        output.moment_of_inertia.I[5] -= mr.y()*r.z();
    }

};


/**
 * @brief 
 * 
 */
template<MOMENT_CONSTANTS NDEG>
class Body : Fixed_Size_Dynamics< NDEG>
{
protected:

    double _time;

    Eigen::Vector3d _position;

    Eigen::Vector3d _velocity;

    Eigen::Vector3d _acceleration;

    Eigen::Quaterniond _orientation;

    Eigen::Vector3d _angular_velocity;

    Eigen::Vector3d _angular_acceleration;

    Inertia<NDEG> _inertia;

    Inertia<NDEG> _inertia_rate;

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