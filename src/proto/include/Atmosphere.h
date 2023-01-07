#pragma once

#include <vector>

#include "WindHistory.h"

struct AltitudeValues {
    double pressure;
    double temperature;
    double inv_sound_speed;
    double density;
    double dynamic_viscosity;
    double gravity;
};

class AltitudeTable {

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

    std::vector<double> grav_table; // TODO: move

    void compute_atmosphere(double maxAlt, double dH, double g0 = 9.806, double R0 = 6371000.0);

public:

    static constexpr double R_GAS = 287.052874;

    static constexpr double AIR_CONST = 287.052874*1.4;

    AltitudeValues* values;

    WindHistory wind;

    AltitudeTable();
    ~AltitudeTable();

    void set_ground(double ground_altitude,
                    double ground_pressure,
                    double ground_temperature,
                    double lapse_rate,
                    double g0 = 9.806,
                    double R0 = 6371000.0);

    void set(double alt, double time);
};

