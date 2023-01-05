#include "../include/Sensors.h"

#include "../include/SingleStageRocket.h"

#include "../../../lib/Eigen/Dense"

#include <cmath>

Sensors::Sensors() {}

void Sensors::get_measured_quantities(const SingleStageRocket& rocket) {

    this->angular_velocity_measured = rocket.angular_velocity;
    this->acceleration_measured = rocket.acceleration;

    for(int i = 0; i < 3; i++) {
        this->angular_velocity_measured.data[i] += this->gyro_variance(this->generator);
        this->acceleration_measured.data[i] += this->accel_variance(this->generator);
    }

    this->dynamic_pressure_measured = this->dynamic_pressure_real + this->pressure_variance(this->generator);

    this->static_pressure_measured = rocket.air_pressure + this->pressure_variance(this->generator);

    this->temperature_measured = rocket.air.temperature;
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

    dobule
}

void Sensors::EKF(const SingleStageRocket& rocket, double dt) {
}

void Sensors::get_computed_quantities(const SingleStageRocket& rocket, double time) {
    this->get_measured_quantities(rocket);

    double dt = time - this->time_old;

    this->my_filter(rocket,dt);

    this->time_old = time;
}
