#include "../include/Filter.h"

#include "../include/Sensors.h"

void FilterNone::update(const Sensors& sensors, double t)
{
    auto dt = t - t_old;
    this->t_old = t;
    const auto& measured = sensors.get_measured_quantities();
    this->computed_state.position += this->computed_state.velocity * dt;
    this->computed_state.velocity += measured.acceleration * dt;

    Vector angle_axis = measured.angular_velocity * dt;
    double angle = angle_axis.norm();
    if(angle > 1e-6)
    {
        this->computed_state.orientation = this->computed_state.orientation* Quaternion(angle,angle_axis); // TODO: check
    }
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
