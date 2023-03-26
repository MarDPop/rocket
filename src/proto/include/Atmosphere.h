#pragma once

#include <vector>

#include "WindHistory.h"

struct AltitudeValues
{
    double pressure = 101325; // Pa
    double temperature = 293.15; // K
    double inv_sound_speed = 0.00291346724; // s/m
    double density = 1.225; // kg/m3
    double dynamic_viscosity = 1.803e-7; // Pa s
    double gravity = 9.806; // m/s2
};

class Atmosphere
{
public:

    static constexpr double R_GAS = 8.31446261815324;

    static constexpr double R_AIR = 287.052874;

    static constexpr double AIR_CONST = 287.052874*1.4;

    AltitudeValues values;

    WindHistory wind;

    Atmosphere();
    virtual ~Atmosphere();

    virtual void set(double alt, double time);

};

class AtmosphereTable : public virtual Atmosphere
{

    /**
    * ground altitude from MSL (m)
    */
    double ground_altitude;

    /**
    * Ground measured pressure (Pa)
    */
    double ground_pressure;

    /**
    * Ground measured temperature (K)
    */
    double ground_temperature;

    /**
    * Lapse rate (K/m), assumed constant for flight (invalid after 10km)
    */
    double lapse_rate;

    /**
    * 1 / height increment of table (m ^ -1)
    */
    double inv_dH;

    /**
    * max altitude (m)
    */
    double maxAlt;

    /**
    * index of last point
    */
    int nAlt;

    // values are precomputed to speed processing
    std::vector<AltitudeValues> air_table;

    std::vector<AltitudeValues> delta_air_table;

    std::vector<double> altitudes;

    void compute_atmosphere(double maxAlt, double dH, double g0 = 9.806, double R0 = 6371000.0);

public:

    AtmosphereTable();
    ~AtmosphereTable();

    void set_ground(double ground_altitude,
                    double ground_pressure,
                    double ground_temperature,
                    double lapse_rate,
                    double g0 = 9.806,
                    double R0 = 6371000.0);

    void set(double alt, double time) override;
};

