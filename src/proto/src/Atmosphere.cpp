#include "../include/Atmosphere.h"

#include <algorithm>
#include <cmath>

AltitudeTable::AltitudeTable() {}

AltitudeTable::~AltitudeTable() {}

void AltitudeTable::set_ground(double ground_altitude,
                               double ground_pressure,
                               double ground_temperature,
                               double lapse_rate,
                               double g0,
                               double R0) {
    this->ground_altitude = ground_altitude;
    this->ground_pressure = ground_pressure;
    this->ground_temperature = ground_temperature;
    this->lapse_rate = lapse_rate;

    this->compute_atmosphere(30000,1,g0,R0);
}

void AltitudeTable::compute_atmosphere(double maxAlt, double dH, double g0, double R0) {
    this->inv_dH = 1.0/dH;
    this->maxAlt = maxAlt;
    unsigned tmp = static_cast<unsigned>(maxAlt/dH) + 1;
    this->air_table.reserve(tmp);
    this->grav_table.reserve(tmp);
    R0 += 0.5*dH;

    double pressure = this->ground_pressure;

    for(double h = 0; h < maxAlt; h += dH){
        AltitudeValues vals;
        vals.pressure = pressure;
        vals.temperature = this->ground_temperature + h*lapse_rate;
        vals.density = pressure/(R_GAS*vals.temperature);
        vals.inv_sound_speed = 1.0/sqrt(AIR_CONST*vals.temperature);
        vals.dynamic_viscosity = 1.458e-6*vals.temperature*sqrt(vals.temperature)/(vals.temperature + 110.4); // sutherland's https://www.cfd-online.com/Wiki/Sutherland%27s_law

        double r = R0/(R0 + h);
        vals.gravity = g0*r*r;
        this->air_table.push_back(vals);

        pressure -= vals.gravity*vals.density*dH;
    }

    this->nAlt = this->air_table.size()-1;
}

void AltitudeTable::set(double alt, double time) {
    int idx = static_cast<int>(alt * this->inv_dH);
    idx = std::clamp(idx,0,this->nAlt);

    this->values = &this->air_table[idx];

    this->wind.set(time);
}
