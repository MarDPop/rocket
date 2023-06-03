#pragma once

#include "../../lib/Eigen/Dense"

// NOT THREAD SAFE, intentionally
/**
 * @brief 
 * 
 */
class Gravity
{
    /**
     * @brief 
     * 
     */
    Eigen::Vector3d acceleration;

public:

    virtual void set_JD(double jd) {}

    virtual const Eigen::Vector3d& get_acceleration(const Eigen::Vector3d& position) const = 0;

};