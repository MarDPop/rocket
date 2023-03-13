#include "Navigation.h"

KinematicState Navigation::get_estimated_state(const SingleStageRocket& rocket, double time)
{
    this->sensors->update(*this->rocket, time);

    this->filter->update(*this->sensors, time);

    return this->filter->get_computed_state();
}
