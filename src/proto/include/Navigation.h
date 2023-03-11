#pragma once

#include "Sensors.h"
#include "Filter.h"
#include "Kinematics.h"

#include <memory>

class Navigation
{

    std::unique_ptr<Filter> filter;

    std::unique_ptr<Sensors> sensors;

public:

    Navigation();
    virtual ~Navigation();

    virtual KinematicState get_estimated_state(const SingleStageRocket& rocket, double time) = 0;
};
