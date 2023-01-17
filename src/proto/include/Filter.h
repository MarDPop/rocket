#pragma once

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

class Sensors;

struct State_Filter
{
    Vector position;
    Vector velocity;

    Quaternion orientation;
    Vector angular_velocity;
};

class Filter
{
protected:

    State_Filter computed_state;

public:

    virtual void update(const Sensors& sensors, double t) = 0;

    inline const State_Filter& get_computed_state() const {
        return this->computed_state;
    }

    inline const Vector& get_computed_position() const {
        return this->computed_state.position;
    }

    inline const Vector& get_computed_velocity() const {
        return this->computed_state.velocity;
    }

    inline Axis get_computed_CS() const {
        return (this->computed_state.orientation.to_rotation_matrix());
    }

    inline const Vector& get_computed_angular_rate() const {
        return this->computed_state.angular_velocity;
    }
};

class FilterNone: public virtual Filter
{

    double t_old;

public:

    void update(const Sensors& sensors, double t) override;

};

class FilterBasic: public virtual Filter
{

    double t_old;

    State_Filter state_old;

public:

    void update(const Sensors& sensors, double t) override;

};

class FilterQuadraticSmooth: public virtual Filter
{

    double t_old;

    State_Filter state_old;

public:

    void update(const Sensors& sensors, double t) override;

};


class FilterMarius : public virtual Filter {

    std::array<Vector,5> last_accel_measurements;

public:

    void update(const Sensors& sensors, double t) override;

};
