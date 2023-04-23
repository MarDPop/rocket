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
    acceleration_inertial.z -= 9.806;

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
    auto dt = time - t_old;
    if(fabs(dt) < 1e-2)
    {
        return;
    }
    this->t_old = time;

    const auto& measured = sensors.get_measured_quantities();
    const auto& computed = sensors.get_computed_quantities();

    double dHdt = (computed.altitude - this->alt_old)/dt;
    this->alt_old = computed.altitude;

    this->computed_state.acceleration = this->computed_state.CS.transpose_mult(measured.acceleration);
    this->computed_state.acceleration.z -= 9.806;

    this->computed_state.position += (this->computed_state.velocity * dt);
    this->computed_state.velocity += (this->computed_state.acceleration * dt);

    this->computed_state.position.z = (this->computed_state.position.z + computed.altitude)*0.5;

    double velocity_factor = fabs((dHdt - this->computed_state.velocity.z)/this->computed_state.velocity.z);
    velocity_factor = velocity_factor/(velocity_factor + 0.2);
    this->computed_state.velocity.z = this->computed_state.velocity.z*velocity_factor + dHdt*(1.0 - velocity_factor);

    Vector angular_velocity = this->computed_state.CS.transpose_mult(measured.angular_velocity);
    double angular_rate = angular_velocity.norm();
    if(angular_rate > 1e-9)
    {
        Axis rotm(angular_rate*dt,angular_velocity *(1/angular_rate));
        this->computed_state.CS = rotm * this->computed_state.CS;
        //this->computed_state.CS.gram_schmidt_orthogonalize();
    }
}

void FilterQuadraticSmooth::update(const Sensors& sensors, double time)
{

}

void FilterMarius::update(const Sensors& sensors, double time)
{

}
