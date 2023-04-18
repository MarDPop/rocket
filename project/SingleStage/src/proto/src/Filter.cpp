#include "../include/Filter.h"

#include "../include/Sensors.h"

void Filter::init(const KinematicState& calibrated_state, double time)
{
    this->computed_state = calibrated_state;
}

void FilterSimpleIntegrate::update(const Sensors& sensors, double time)
{
    auto dt = time - t_old;
    if(fabs(dt) < 1e-8)
    {
        return;
    }
    this->t_old = time;
    const auto& measured = sensors.get_measured_quantities();
    this->computed_state.position += this->computed_state.velocity * dt;
    this->computed_state.velocity += this->computed_state.CS.transpose_mult(measured.acceleration * dt);

    Vector angle_axis = this->computed_state.CS.transpose_mult(measured.angular_velocity * dt);
    double angle = angle_axis.norm();
    if(angle > 1e-6)
    {
        Axis rotm(angle,angle_axis * (1.0/angle));
        this->computed_state.CS = rotm * this->computed_state.CS; // TODO: check
    }
}

void FilterBasic::update(const Sensors& sensors, double time)
{

}

void FilterQuadraticSmooth::update(const Sensors& sensors, double time)
{

}

void FilterMarius::update(const Sensors& sensors, double time)
{

}
