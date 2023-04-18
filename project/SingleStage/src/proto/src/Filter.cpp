#include "../include/Filter.h"

#include "../include/Sensors.h"

void Filter::init(const KinematicState& calibrated_state, double time)
{
    this->computed_state = calibrated_state;
}


void Filter::update(const Sensors& sensors, double time) {}


void FilterSimpleIntegrate::update(const Sensors& sensors, double time)
{
    auto dt = time - t_old;
    if(fabs(dt) < 1e-8)
    {
        return;
    }
    this->t_old = time;
    const auto& measured = sensors.get_measured_quantities();
    Axis body2inertial = this->computed_state.CS.get_transpose();
    Vector acceleration_inertial = body2inertial*measured.acceleration;
    acceleration_inertial.z += 9.806;

    this->computed_state.position += this->computed_state.velocity * dt;
    this->computed_state.velocity += acceleration_inertial * dt;

    Vector angle_axis = body2inertial * (measured.angular_velocity * dt);
    double angle = angle_axis.norm();
    if(angle > 1e-10)
    {
        Axis rotm(angle,angle_axis / angle);
        this->computed_state.CS = rotm * body2inertial;
        this->computed_state.CS.transpose();
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
