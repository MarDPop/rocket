#pragma once

#include "../../../lib/Eigen/Dense"

class Vehicle;

class Component
{
protected:

    Vehicle* vehicle = nullptr;

    Eigen::Vector3d center;

    double mass;

    Eigen::Matrix3d inertiaMatrix;

public:

    Component();
    ~Component();

    inline void set_vehicle(Vehicle* vehicle)
    {
        this->vehicle = vehicle;
    }

};
