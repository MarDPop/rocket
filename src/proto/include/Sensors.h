#pragma once

#include <random>

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

class SingleStageRocket;

class Sensors {

    double sigma_pressure = 0.5; // Pa

    double sigma_gyro = 5e-4; // rad/s

    double sigma_accelerometer = 1e-2; // m/s2

    double sigma_temperature = 1e-2; // Kelvin

    std::default_random_engine generator(clock());

    normal_distribution<double> pressure_variance(0.0, sigma_pressure);

    normal_distribution<double> gyro_variance(0.0, sigma_gyro);

    normal_distribution<double> accel_variance(0.0, sigma_accelerometer);

    double static_pressure_measured;

    double dynamic_pressure_measured;

    double total_pressure_measured;

    double temperature_measured;

    double mach_measured;

    Vector acceleration_measured;

    Vector angular_velocity_measured;

    double time_old;

    Vector estimated_position;

    Vector estimated_velocity;

    Vector estimated_acceleration;

    Quaternion estimated_orientation;

    kalman_filter(const SingleStageRocket& rocket, double dt);

    my_filter(const SingleStageRocket& rocket, double dt);

    EKF(const SingleStageRocket& rocket, double dt);

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

    inline double get_measured_dynamic_pressure() {
        return this->dynamic_pressure_measured;
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
