#pragma once

#include "../../Eigen/Dense"

class Geometry
{

public:

    virtual bool inside(const Eigen::Vector3d& point)
    {
        return false;
    }
};

class Geometry_Sphere 
{

};