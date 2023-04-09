#include "../include/Navigation.h"

const KinematicState& Navigation::get_estimated_state(const SingleStageRocket& rocket, double time)
{
    this->sensors->update(rocket, time);

    this->filter->update(*this->sensors, time);

    return this->filter->get_computed_state();
}
