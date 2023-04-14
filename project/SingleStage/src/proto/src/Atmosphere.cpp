#include "../include/Atmosphere.h"

#include <algorithm>
#include <cmath>

Atmosphere::Atmosphere(){}

Atmosphere::~Atmosphere(){}

void Atmosphere::set(double alt, double time)
{

}

AtmosphereTable::AtmosphereTable(double _ground_altitude,
                               double _ground_pressure,
                               double _ground_temperature,
                               double _lapse_rate,
                               double g0,
                               double R0) :
                                ground_altitude(_ground_altitude),
                                ground_pressure(_ground_pressure),
                                ground_temperature(_ground_temperature),
                                lapse_rate(_lapse_rate)
{
    this->compute_atmosphere(30000,1,g0,R0);
}

AtmosphereTable::~AtmosphereTable() {}

void AtmosphereTable::compute_atmosphere(double maxAlt, double dH, double g0, double R0)
{
    this->inv_dH = 1.0/dH;
    this->maxAlt = maxAlt;
    unsigned tmp = static_cast<unsigned>(maxAlt/dH) + 1;
    this->air_table.reserve(tmp);
    this->altitudes.reserve(tmp);
    R0 += 0.5*dH;

    double pressure = this->ground_pressure;

    for(double h = 0; h < maxAlt; h += dH)
    {
        AltitudeValues vals;
        vals.pressure = pressure;
        vals.temperature = this->ground_temperature + h*lapse_rate;
        vals.density = pressure/(R_AIR*vals.temperature);
        vals.inv_sound_speed = 1.0/sqrt(AIR_CONST*vals.temperature);
        vals.dynamic_viscosity = 1.458e-6*vals.temperature*sqrt(vals.temperature)/(vals.temperature + 110.4); // sutherland's https://www.cfd-online.com/Wiki/Sutherland%27s_law

        double r = R0/(R0 + h);
        vals.gravity = g0*r*r;
        this->air_table.push_back(vals);
        this->altitudes.push_back(h);

        pressure -= vals.gravity*vals.density*dH;
    }

    this->nAlt = this->air_table.size()-1;

    this->delta_air_table.resize(this->nAlt);

    for(int idx = 0; idx < this->nAlt; idx++)
    {
        const auto& lo = this->air_table[idx];
        const auto& hi = this->air_table[idx+1];
        auto& delta = this->delta_air_table[idx];
        delta.pressure = (hi.pressure - lo.pressure)*this->inv_dH;
        delta.temperature = (hi.temperature - lo.temperature)*this->inv_dH;
        delta.inv_sound_speed = (hi.inv_sound_speed - lo.inv_sound_speed)*this->inv_dH;
        delta.density = (hi.density - lo.density)*this->inv_dH;
        delta.dynamic_viscosity = (hi.dynamic_viscosity - lo.dynamic_viscosity)*this->inv_dH;
        delta.gravity = (hi.gravity - lo.gravity)*this->inv_dH;
    }
}

void AtmosphereTable::set(double alt, double time)
{
    if(alt > this->altitudes.back())
    {
        this->values = this->air_table.back();
        return;
    }

    if(alt < this->altitudes.front())
    {
        this->values = this->air_table.front();
        return;
    }

    int idx = static_cast<int>(alt * this->inv_dH);

    const auto& lo = this->air_table[idx];
    const auto& delta = this->delta_air_table[idx];

    double dH = alt - this->altitudes[idx];

    this->values.pressure = lo.pressure + delta.pressure*dH;
    this->values.temperature = lo.temperature + delta.temperature*dH;
    this->values.inv_sound_speed = lo.inv_sound_speed + delta.inv_sound_speed*dH;
    this->values.density = lo.density + delta.density*dH;
    this->values.dynamic_viscosity = lo.dynamic_viscosity + delta.dynamic_viscosity*dH;
    this->values.gravity = lo.gravity + delta.gravity*dH;

    this->wind.set(time);
}
