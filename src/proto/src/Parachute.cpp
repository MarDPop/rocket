#include "../include/Parachute.h"

Parachute::Parachute(SingleStageRocket& _rocket) : rocket(_rocket) {}

Parachute::Parachute(SingleStageRocket& _rocket, double _CDA) : rocket(_rocket), CDA(_CDA) {}

Parachute::~Parachute() {}

void Parachute::update()
{

    this->tether_force = this->rocket.aerodynamics->aero_values.unit_v_air * (this->CDA*this->rocket.aerodynamics->aero_values.dynamic_pressure);

}
