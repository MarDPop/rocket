#include "../include/Parachute.h"

#include "../include/SingleStageRocket.h"

Parachute::Parachute(SingleStageRocket& _rocket) : rocket(_rocket) {}

Parachute::Parachute(SingleStageRocket& _rocket, double _CDA) : rocket(_rocket), CDA(_CDA) {}

Parachute::~Parachute() {}

const BodyAction& Parachute::update(double time)
{
    const auto& air_values = this->rocket.get_aerodynamics().get_aero_values();
    this->tether_force = air_values.unit_v_air * (this->CDA*air_values.dynamic_pressure);
}
