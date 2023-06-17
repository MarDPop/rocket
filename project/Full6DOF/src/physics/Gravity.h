#pragma once

#include "../../lib/Eigen/Dense"

#include "Time.h"

#include <vector>
#include <memory>

/**
 * @brief 
 * 
 */
class Gravity
{

public:

    virtual void set_time(EpochTime time) {}

    virtual void get_acceleration(const Eigen::Vector3d& position, double* acceleration) const = 0;

    inline Eigen::Vector3d get_acceleration(const Eigen::Vector3d& position)
    {
        Eigen::Vector3d acceleration;
        this->get_acceleration(position,acceleration.data());
        return acceleration;
    }

};


class Multi_Gravity : public virtual Gravity
{

    std::vector<std::unique_ptr<Gravity>> _gravities;

public:

    inline Multi_Gravity() {}

    inline void add_gravity(std::unique_ptr<Gravity>& gravity)
    {
        this->_gravities.push_back(std::move(gravity));
    }


};