#include "../include/Filter.h"

#include "../include/Sensors.h"

void FilterNone::update(const Sensors& sensors, double t)
{
    auto dt = t - t_old;
    this->t_old = t;
    const auto& measured = sensors.get_measured_quantities();
    this->computed_state.position += this->computed_state.velocity * dt;
    this->computed_state.velocity += measured.acceleration * dt;
}

void FilterBasic::update(const Sensors& sensors, double t)
{

}

void FilterQuadraticSmooth::update(const Sensors& sensors, double t)
{

}

void FilterMarius::update(const Sensors& sensors, double t)
{

}
