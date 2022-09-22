#pragma once

#include "../../common/include/Cartesian.h"

class Vehicle;

class Component {
protected:

    Vehicle* vehicle = nullptr;

public:

    Cartesian::Vector center;

    double mass;

    inline void set_vehicle(Vehicle* vehicle) {
        this->vehicle = vehicle;
    }

};
