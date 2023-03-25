#pragma once

#include "Sensors.h"
#include "Filter.h"
#include "Kinematics.h"

#include <memory>

class Navigation
{
protected:

    KinematicState estimated_state;

    std::unique_ptr<Filter> filter;

    std::unique_ptr<Sensors> sensors;

public:

    Navigation();
    virtual ~Navigation();

    virtual const KinematicState& get_estimated_state(const SingleStageRocket& rocket, double time);
};
