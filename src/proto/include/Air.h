#pragma once

#include <vector>

#include "WindHistory.h"

struct AirVals {
    double pressure;
    double temperature;
    double inv_sound_speed;
    double density;
    double dynamic_viscosity;
    double gravity;
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

    std::vector<double> grav_table; // TODO: move

    AirVals* properties;

    /**
    * current air speed (m/s)
    */
    double airspeed;

    /**
    * current velocity with respect unit vector
    */
    Vector unit_v_air;

    /**
    * current mach
    */
    double mach;

    /**
    * current compressible dynamic pressure (Pa)
    */
    double dynamic_pressure;

    void compute_atmosphere(double maxAlt, double dH, double g0 = 9.806);

public:

    static constexpr double R_GAS = 287.052874;

    static constexpr double AIR_CONST = 287.052874*1.4;

    WindHistory wind;

    Air();
    ~Air();

    inline double get_airspeed() {
        return this->airspeed;
    }

    inline double get_dynamic_pressure() const {
        return this->dynamic_pressure;
    }

    inline double get_temperature() const {
        return this->properties->temperature;
    }

    inline double get_static_pressure() const {
        return this->properties->pressure;
    }

    inline double get_inv_sound_speed() const {
        return this->properties->inv_sound_speed;
    }

    inline double get_density() const {
        return this->properties->density;
    }

    inline double get_dynamic_viscosity() const {
        return this->properties->dynamic_viscosity;
    }

    inline double get_mach() const {
        return this->mach;
    }

    inline const Vector& get_air_velocity_unit_vector() const {
        return this->unit_v_air;
    }

    void set_ground(double ground_altitude, double ground_pressure, double ground_temperature, double lapse_rate);

    double compute_vals(double h, const Vector& velocity, double time);

};
