#include "../include/Sensors.h"

#include "../include/SingleStageRocket.h"

#include "../../../lib/Eigen/Dense"

#include <cmath>

Sensors::Sensors() : generator(std::random_device{}()) {
    this->set_sensor_variances(0.5,0.1,0.01,0.0001);
}

Sensors::~Sensors() {}

void Sensors::set_sensor_variances(double sigma_pressure, double sigma_temperature, double sigma_accel, double sigma_gyro) {
    this->barometer_variance = std::normal_distribution<double>(0,sigma_pressure);
    this->thermometer_variance = std::normal_distribution<double>(0,sigma_temperature);
    this->accelerometer_variance = std::normal_distribution<double>(0,sigma_accel);
    this->gyro_variance = std::normal_distribution<double>(0,sigma_gyro);
}

void Sensors::get_measured_quantities(const SingleStageRocket& rocket) {

    this->angular_velocity_computed = rocket.angular_velocity;
    this->acceleration_computed = rocket.acceleration;

    for(int i = 0; i < 3; i++) {
        this->angular_velocity_computed.data[i] += this->gyro_variance(this->generator);
        this->acceleration_computed.data[i] += this->accelerometer_variance(this->generator);
    }

    this->dynamic_pressure_measured = rocket.air.get_dynamic_pressure() + this->barometer_variance(this->generator);

    this->static_pressure_measured = rocket.air.get_static_pressure() + this->barometer_variance(this->generator);

    this->temperature_measured = rocket.air.get_temperature() + this->thermometer_variance(this->generator);
}

void Sensors::kalman_filter(const SingleStageRocket& rocket, double dt) {
    auto F = Eigen::MatrixXd::Zero(6,6);
}

void Sensors::my_filter(const SingleStageRocket& rocket, double dt) {
    double tmp = this->dynamic_pressure_measured / this->static_pressure_measured + 1.0;
    if(tmp < 1.0) {
        tmp = 1.0;
    }
    tmp = pow(tmp, 2/7);

    double measured_mach = sqrt(5*(tmp - 1.0));

}

void Sensors::EKF(const SingleStageRocket& rocket, double dt) {
}

void Sensors::compute_quantities(const SingleStageRocket& rocket, double time) {
    this->get_measured_quantities(rocket);

    double dt = time - this->time_old;

    this->my_filter(rocket,dt);

    this->time_old = time;
}
