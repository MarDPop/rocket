#pragma once

#include "../../common/include/Cartesian.h"

#include "Kinematics.h"
#include "Sensors.h"

using namespace Cartesian;

class Filter
{
protected:

    KinematicState computed_state;

public:

    virtual void init(const KinematicState& calibrated_state, double time);

    virtual void update(const Sensors& sensors, double time) = 0;

    inline const KinematicState& get_computed_state() const {
        return this->computed_state;
    }
};

class FilterSimpleIntegrate: public virtual Filter
{

    double t_old;

public:

    void update(const Sensors& sensors, double t) override;

};

class FilterBasic: public virtual Filter
{

    double t_old;

    KinematicState state_old;

public:

    void update(const Sensors& sensors, double t) override;

};

class FilterQuadraticSmooth: public virtual Filter
{

    double t_old;

    KinematicState state_old;

public:

    void update(const Sensors& sensors, double t) override;

};

class FilterKalman : public virtual Filter
{
    std::array<double,6> state_predicted;

public:

    void update(const Sensors& sensors, double t) override;

};

class FilterMarius : public virtual Filter
{

    std::array<Vector,5> last_accel_measurements;

public:

    void update(const Sensors& sensors, double t) override;

};
