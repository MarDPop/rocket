#include "../include/Parachute.h"

#include "../include/SingleStageRocket.h"

Parachute::Parachute(SingleStageRocket& rocket) : _rocket(rocket)
{
    this->_action.zero();
}

Parachute::Parachute(SingleStageRocket& rocket, double CDA) : _rocket(rocket), _CDA(CDA)
{
    this->_action.zero();
}

Parachute::~Parachute() {}

const BodyAction& Parachute::update(double time)
{
    const auto& air_values = this->_rocket.get_aerodynamics().get_aero_values();
    this->_action.force = air_values.unit_v_air_body * (-this->_CDA*air_values.dynamic_pressure);
    return this->_action;
}

ParachuteTimed::ParachuteTimed(SingleStageRocket& rocket, double CDA, double deployment_time) : Parachute(rocket,CDA),
                                                                                                _deployment_duration(deployment_time) {}

const BodyAction& ParachuteTimed::update(double time)
{
    double dt = time - this->_time_deployed;
    double CDA;
    if(dt > this->_deployment_duration)
    {
        CDA = -this->_CDA;
    }
    else
    {
        CDA = -this->_CDA * dt / this->_deployment_duration;
    }
    const auto& air_values = this->_rocket.get_aerodynamics().get_aero_values();
    this->_action.force = air_values.unit_v_air_body * (CDA*air_values.dynamic_pressure);
    return this->_action;
}

ConstrainedParachute::ConstrainedParachute(SingleStageRocket& rocket) : Parachute(rocket) {}

void ConstrainedParachute::set_params(double area, double tether_length, double youngs_modulus, double area_tether_cross_section)
{
    this->_CDA = area*1.2;
    this->_tether_length = tether_length;
    this->_tether_spring_constant = youngs_modulus*area_tether_cross_section/tether_length;
    this->_volume_deployed = area*sqrt(area)/4.0;
    this->_relative_position.zero();
    this->_relative_velocity.zero();
}

const BodyAction& ConstrainedParachute::update(double time)
{
    double dt = time - this->_time_deployed;
    double CDA, volume;
    if(dt > this->_deployment_duration)
    {
        CDA = -this->_CDA;
        volume = this->_volume_deployed;
    }
    else
    {
        double deployment_factor = dt / this->_deployment_duration;
        CDA = -this->_CDA * deployment_factor;
        volume = this->_volume_deployed * deployment_factor;
    }
    const auto& air_values = this->_rocket.get_aerodynamics().get_aero_values();

    double mass = this->_rocket.get_environment().values.density * volume;

    Vector drag = air_values.unit_v_air_body * (CDA*air_values.dynamic_pressure);

    Vector tension_force;
    double stretched_tether_length = this->_relative_position.norm();
    if(stretched_tether_length > this->_tether_length)
    {
        double tension = (stretched_tether_length - this->_tether_length)*this->_tether_spring_constant;
        CDA *= (this->_tether_length / stretched_tether_length);
        tension_force = this->_relative_position * (tension / )
    }
    else
    {
        tension_force.zero();
    }

    this->_action.force =
    return this->_action;
}
