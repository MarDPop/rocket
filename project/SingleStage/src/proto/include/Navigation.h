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

    inline void set_filter(std::unique_ptr<Filter> filter) {
        this->filter = std::move(filter);
    }

    inline void set_sensors(std::unique_ptr<Sensors> sensors) {
        this->sensors = std::move(sensors);
    }

    const KinematicState& get_estimated_state(const SingleStageRocket& rocket, double time); // make make virtual in future
};
