#pragma once

#include "../common/include/Cartesian.h"
#include <array>

class Gravity {

public:

    Cartesian::Vector acceleration;

    virtual void compute(const double* position, const double& latitude, const double& longitude);
};
