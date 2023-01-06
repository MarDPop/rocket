#pragma once

#include <random>

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

class SingleStageRocket;

class Sensors {

    std::default_random_engine generator;

    std::normal_distribution<double> barometer_variance;

    std::normal_distribution<double> accelerometer_variance;

    std::normal_distribution<double> thermometer_variance;

    std::normal_distribution<double> gyro_variance;

    double static_pressure_measured;

    double dynamic_pressure_measured;

    double total_pressure_measured;

    double temperature_measured;

    double mach_measured;

    double time_old;

    Vector position_computed;

    Vector velocity_computed;

    Vector acceleration_computed;

    Vector angular_velocity_computed;

    Quaternion orientation_computed;

    void kalman_filter(const SingleStageRocket& rocket, double dt);

    void my_filter(const SingleStageRocket& rocket, double dt);

    void EKF(const SingleStageRocket& rocket, double dt);

    void get_measured_quantities(const SingleStageRocket& rocket);

public:

    inline double get_measured_dynamic_pressure() {
        return this->dynamic_pressure_measured;
    }

    inline const Vector& get_computed_angular_rate() {
        return this->angular_velocity_computed;
    }

    inline const Vector& get_computed_position() {
        return this->position_computed;
    }

    inline const Vector& get_computed_velocity() {
        return this->velocity_computed;
    }

    inline const Vector& get_computed_acceleration() {
        return this->acceleration_computed;
    }

    inline const Axis& get_computed_CS() {
        return this->orientation_computed.to_rotation_matrix();
    }

    Sensors();
    ~Sensors();

    void set_sensor_variances(double sigma_pressure, double sigma_temperature, double sigma_accelerometer, double sigma_gyro);

    void init();

    void compute_quantities(const SingleStageRocket& rocket, double time);

};
