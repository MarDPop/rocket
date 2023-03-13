#pragma once

#include "Control.h"
#include "Guidanae.h"
#include "Navigation.h"

#include <memory>

class SingleStageRocket;

class GNC
{
public:

    const SingleStageRocket& rocket;

    std::unique_ptr<Guidance> guidance;

    std::unique_ptr<Navigation> navigation;

    std::unique_ptr<Control> control;

    inline GNC(const SingleStageRocket& _rocket) : rocket(_rocket) {}

    inline void update(double time)
    {
        auto estimated_state = this->navigation->get_estimated_state(this->rocket, time);

        auto desired_state = this->guidance->get_commanded_state(estimated_state, time);

        this->control->get_outputs(desired_state, estimated_state, time);
    }
};
