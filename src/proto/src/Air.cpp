#include "../include/Air.h"

#include <algorithm>
#include <cmath>

Air::Air() {}

Air::~Air() {}

void Air::set_ground(double ground_altitude, double ground_pressure, double ground_temperature, double lapse_rate) {
    this->ground_altitude = ground_altitude;
    this->ground_pressure = ground_pressure;
    this->ground_temperature = ground_temperature;
    this->lapse_rate = lapse_rate;

    this->compute_atmosphere(30000,1);
}

void Air::compute_atmosphere(double maxAlt, double dH) {
    this->inv_dH = 1.0/dH;
    this->maxAlt = maxAlt;
    unsigned tmp = static_cast<unsigned>(maxAlt/dH) + 1;
    this->air_pressure_table.reserve(tmp);
    this->air_density_table.reserve(tmp);
    this->air_sound_speed_table.reserve(tmp);
    this->grav_table.reserve(tmp);
    double R0 = 6371000 + this->ground_altitude + 0.5*dH;

    double pressure = this->ground_pressure;

    for(double h = 0; h < maxAlt; h+=1.0){
        double temperature = this->ground_temperature + h*lapse_rate;
        double density = pressure/(R_GAS*temperature);
        double r = 6371000.0/(R0 + h);
        double g = 9.806*r*r;
        this->air_pressure_table.push_back(pressure);
        this->air_density_table.push_back(density);
        this->air_sound_speed_table.push_back(1.0/sqrt(AIR_CONST*temperature));
        this->grav_table.push_back(g);

        pressure -= g*density*dH; // dz = 1 meter
    }

    this->nAlt = this->air_pressure_table.size()-1;
}

void Air::get_air_properties(double h, const Vector& velocity, double time) {
    int idx = static_cast<int>(h * this->inv_dH);
    idx = std::clamp(idx,0,this->nAlt);
    this->air_density = this->air_density_table[idx];
    this->air_pressure = this->air_pressure_table[idx];
    this->sound_speed_inv = this->air_sound_speed_table[idx];
    this->grav = this->grav_table[idx];

    this->wind.set(time);

    this->air_velocity = velocity - this->wind.wind;

    this->airspeed = this->air_velocity.norm();

    this->unit_v_air = this->air_velocity * (1.0/this->airspeed);

    this->mach = this->airspeed * this->sound_speed_inv;

    double tmp = 1.0 + 0.2*this->mach*this->mach;

    this->dynamic_pressure = this->air_pressure*(tmp*tmp*tmp*sqrt(tmp) - 1.0);
}
