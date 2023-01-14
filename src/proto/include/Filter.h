#pragma once

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

class Sensors;

struct State_Filter {
    Vector position;
    Vector velocity;
    Vector acceleration;
    Vector angular_velocity;
    Quaternion orientation;
};

class Filter {

    double t_old;

    State_Filter state;

public:

    virtual void update(const Sensors& sensors, double t) = 0;

    inline const Vector& get_computed_angular_rate() {
        return this->state.angular_velocity;
    }

    inline const Vector& get_computed_position() {
        return this->state.position;
    }

    inline const Vector& get_computed_velocity() {
        return this->state.velocity;
    }

    inline const Vector& get_computed_acceleration() {
        return this->state.acceleration;
    }

    inline const Axis& get_computed_CS() {
        return this->state.orientation.to_rotation_matrix();
    }
};

class FilterMarius : public virtual Filter {

    std::array<Vector,5> last_accel_measurements;

public:

    void update(const Sensors& sensors, double t) override;

};
