#pragma once

#include <random>

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

class SingleStageRocket;

class Sensors {

    constexpr double sigma_pressure = 0.5; // Pa

    constexpr double sigma_gyro = 5e-4; // rad/s

    constexpr double sigma_accelerometer = 1e-2; // m/s2

    std::default_random_engine generator(clock());

    normal_distribution<double> pressure_variance(0.0, sigma_pressure);

    normal_distribution<double> gyro_variance(0.0, sigma_gyro);

    normal_distribution<double> accel_variance(0.0, sigma_accelerometer);

    double static_pressure_measured;

    double dynamic_pressure_measured;

    Vector acceleration_measured;

    Vector angular_velocity_measured;

    double dynamic_pressure_real;

    Vector air_velocity_real;

    double air_speed_sq_real;

    double ascent_rate_computed;

    double dynamic_pressure_computed;

    Vector angular_velocity_computed;

    Axis CS_computed;

    double time_old;

    kalman_filter(const SingleStageRocket& rocket, double dt);

    get_measured_quantities(const SingleStageRocket& rocket);

public:

    inline double get_real_airspeed() {
        return sqrt(this->air_speed_sq_real);
    }

    inline Vector get_real_air_velocity() {
        return this->air_velocity_real;
    }

    inline double get_real_dynamic_pressure() {
        return this->dynamic_pressure_real;
    }

    inline double get_real_dynamic_pressure() {
        return this->dynamic_pressure_real;
    }

    inline double get_computed_dynamic_pressure() {
        return this->dynamic_pressure_computed;
    }

    inline double get_computed_ascent_rate() {
        return this->ascent_rate_computed;
    }

    inline Axis get_computed_angular_rate() {
        return this->angular_velocity_computed;
    }

    inline Axis get_computed_CS() {
        return this->CS_computed;
    }

    Sensors();
    ~Sensors();

    init();

    compute_quantities(const SingleStageRocket& rocket, double time);

};
