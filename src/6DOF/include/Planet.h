#pragma once

#include "Atmosphere.h"
#include "Gravity.h"
#include "Geoid.h"
#include <memory>
#include "../../common/include/Cartesian.h"
#include "../../common/include/Time.h"

class Vehicle;

using namespace Cartesian;

union Geodetic {
    double v[3];
    struct {
        double latitude;
        double longitude;
        double altitude;
    };
};

class Planet {

public:

    std::unique_ptr< Atmosphere > atmosphere;

    std::unique_ptr< Gravity > gravity;

    std::unique_ptr< Geoid > geoid;

    Axis ECEF;

    Axis ENU; // east north up in ECI

    Geodetic geodetic;

    Spherical spherical;

    Vector body_fixed_pos;

    Vector body_fixed_velocity;

    Time t_ref;

    void update(Vehicle* vehicle, double time);

};
