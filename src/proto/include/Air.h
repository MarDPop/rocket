#pragma once

#include <vector>

#include "WindHistory.h"

class Air {

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

    /* ATMOSPHERE TABLE */

    double inv_dH;

    double maxAlt;

    int nAlt;

    // values are precomputed to speed processing
    // values for every meter
    // nearest neighbor is used for interpolation
    std::vector<double> air_density_table;
    std::vector<double> air_pressure_table;
    std::vector<double> air_sound_speed_table;
    std::vector<double> grav_table;

public:

    static constexpr double R_GAS = 287.052874;

    static constexpr double AIR_CONST = 287.052874*1.4;

    Vector air_velocity;

    /**
    * current air pressure (Pa)
    */
    double air_pressure;

    /**
    * current air density (kg/m3)
    */
    double air_density;

    /**
    * current inverse of sound speend  ((m/s) ^ -1)
    */
    double sound_speed_inv;

    /**
    * current air speed (m/s)
    */
    double airspeed;

    /**
    * current mach
    */
    double mach;

    /**
    * current compressible dynamic pressure (Pa)
    */
    double dynamic_pressure;

    Vector unit_v_air;

    WindHistory wind;

    Air();
    ~Air();

    void set_ground(double ground_altitude, double ground_pressure ,double ground_temperature, double lapse_rate);

    void compute_atmosphere();

    void get_air_properties(double h, const Vector& velocity, double time);

};
