#pragma once

#include "Atmosphere.h"
#include "Gravity.h"
#include <memory>
#include "../../common/include/Planet.h"
#include "../../common/include/Cartesian.h"
#include "../../common/include/Time.h"

class Vehicle;

using namespace Cartesian;

struct Body_Reference {

    std::unique_ptr< Atmosphere > atmosphere;

    std::unique_ptr< Gravity > gravity;

    std::unique_ptr< Planet > planet;

    Axis body_fixed_CS;

    Axis ENU; // east north up in ECI

    Geodetic geodetic;

    Spherical spherical;

    Vector body_fixed_pos;

    Vector body_fixed_velocity;

    Time t_ref;

    void update(Vehicle* vehicle, double time);

};
