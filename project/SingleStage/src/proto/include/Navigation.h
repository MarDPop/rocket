#pragma once

#include "Sensors.h"
#include "Filter.h"
#include "Kinematics.h"

#include <memory>

struct Navigation
{
    std::unique_ptr<Filter> filter;

    std::unique_ptr<Sensors> sensors;

    inline const KinematicState& get_estimated_state(const SingleStageRocket& rocket, double time)
    {
        this->sensors->update(rocket, time);

        this->filter->update(*this->sensors, time);

        return this->filter->get_computed_state();
    }
};
