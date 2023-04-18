#pragma once

#include "Sensors.h"
#include "Filter.h"
#include "Kinematics.h"

#include <memory>

class Navigation
{
protected:

    KinematicState estimated_state;

public:

    std::unique_ptr<Filter> filter;

    std::unique_ptr<Sensors> sensors;

    Navigation();
    virtual ~Navigation();

    const KinematicState& get_estimated_state(const SingleStageRocket& rocket, double time); // make make virtual in future
};
