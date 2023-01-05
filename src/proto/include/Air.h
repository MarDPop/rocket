#pragma once

#include <vector>

#include "WindHistory.h"

struct AirVals {
    double pressure;
    double temperature;
    double inv_sound_speed;
    double density;
    double absolute_viscosity;
};

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
    std::vector<AirVals> air_table;

    AirVals* properties;

    Vector air_velocity;

    /**
    * current air speed (m/s)
    */
    double airspeed;

    Vector unit_v_air;

    /**
    * current mach
    */
    double mach;

    /**
    * current compressible dynamic pressure (Pa)
    */
    double dynamic_pressure;

public:

    static constexpr double R_GAS = 287.052874;

    static constexpr double AIR_CONST = 287.052874*1.4;

    inline double get_airspeed() {
        return this->airspeed;
    }

    WindHistory wind;

    Air();
    ~Air();

    void set_ground(double ground_altitude, double ground_pressure ,double ground_temperature, double lapse_rate);

    void compute_atmosphere();

    double set_altitude(double h, const Vector& velocity, double time);

};
