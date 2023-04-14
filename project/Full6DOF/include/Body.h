#pragma once

#include "Atmosphere.h"
#include "Gravity.h"
#include <memory>
#include "../../common/include/Planet.h"
#include "../../common/include/Time.h"

#include "../../../lib/Eigen/Dense"

class Vehicle;

struct Body_Reference {

    std::unique_ptr< Atmosphere > atmosphere;

    std::unique_ptr< Gravity > gravity;

    std::unique_ptr< Planet > planet;

    Eigen::Matrix3d body_fixed_CS;

    Eigen::Matrix3d ENU; // east north up in ECI

    Geodetic geodetic;

    Spherical spherical;

    Eigen::Vector3d body_fixed_pos;

    Eigen::Vector3d body_fixed_velocity;

    Time t_ref;

    void update(Vehicle* vehicle, double time);

};
