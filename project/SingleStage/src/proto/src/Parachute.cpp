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
