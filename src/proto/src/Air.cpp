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

void Air::compute_atmosphere(double maxAlt, double dH, double g0) {
    this->inv_dH = 1.0/dH;
    this->maxAlt = maxAlt;
    unsigned tmp = static_cast<unsigned>(maxAlt/dH) + 1;
    this->air_table.reserve(tmp);
    this->grav_table.reserve(tmp);
    double R0 = 6371000 + this->ground_altitude + 0.5*dH;

    double pressure = this->ground_pressure;

    for(double h = 0; h < maxAlt; h+=1.0){
        AirVals vals;
        vals.pressure = pressure;
        vals.temperature = this->ground_temperature + h*lapse_rate;
        vals.density = pressure/(R_GAS*vals.temperature);
        vals.inv_sound_speed = 1.0/sqrt(AIR_CONST*vals.temperature);
        vals.dynamic_viscosity = 1.458e-6*vals.temperature*sqrt(vals.temperature)/(vals.temperature + 110.4); // sutherland's https://www.cfd-online.com/Wiki/Sutherland%27s_law

        this->air_table.push_back(vals);

        double r = 6371000.0/(R0 + h);
        double g = g0*r*r;
        this->grav_table.push_back(g);

        pressure -= g*vals.density*dH; // dz = 1 meter
    }

    this->nAlt = this->air_table.size()-1;
}

double Air::compute_vals(double h, const Vector& velocity, double time) {
    int idx = static_cast<int>(h * this->inv_dH);
    idx = std::clamp(idx,0,this->nAlt);

    this->properties = &this->air_table[idx];

    this->wind.set(time);

    Vector air_velocity = velocity - this->wind.wind;

    this->airspeed = air_velocity.norm();

    this->unit_v_air = air_velocity * (1.0/this->airspeed);

    this->mach = this->airspeed * this->properties->inv_sound_speed;

    double tmp = 1.0 + 0.2*this->mach*this->mach;

    this->dynamic_pressure = this->properties->pressure*(tmp*tmp*tmp*sqrt(tmp) - 1.0);

    return this->grav_table[idx];
}
