#pragma once

#include "../../../lib/Eigen/Dense"

struct Action
{
    Eigen::Vector3d force;
    Eigen::Vector3d moment;
    Eigen::Vector3d center;

    virtual void update(double time) = 0;
};
