#include "../include/Parachute.h"

Parachute::Parachute(SingleStageRocket& _rocket) : rocket(_rocket) {}

Parachute::~Parachute() {}

void Parachute::update()
{
    double current_tether_length = this->chute_position.norm();
    if(current_tether_length < this->tether_length)
    {
        this->tether_force.zero();
        return;
    }
    double dx = current_tether_length - this->tether_length;
    double tension = dx*this->tether_tension_coefficient;
    this->tether_force = this->chute_position * (tension / current_tether_length);

    double area;
    double air_volume;
    if(this->fully_deployed)
    {
        area = this->area[1];
        air_volume = this->air_volume[1];
    }
    else
    {

    }

    double parachute_air_mass = air_volume*this->rocket.altitude_table.values->density;


}
