#include "../include/Sensors.h"

#include "../include/SingleStageRocket.h"

#include "../../../lib/Eigen/Dense"

Sensors::Sensors() {}

void Sensors::get_measured_quantities(const SingleStageRocket& rocket) {

    this->airspeed_real = rocket.velocity - rocket.wind.wind;

    this->air_speed_sq_real = this->airspeed_real.dot(this->airspeed_real);

    this->dynamic_pressure_real = 0.5*rocket.air_density*this->air_speed_sq_real;

    this->angular_velocity_measured = rocket.angular_velocity;
    this->acceleration_measured = rocket.acceleration;

    for(int i = 0; i < 3; i++) {
        this->angular_velocity_measured.data[i] += this->gyro_variance(this->generator);
        this->acceleration_measured.data[i] += this->accel_variance(this->generator);
    }

    this->dynamic_pressure_measured = this->dynamic_pressure_real + this->pressure_variance(this->generator);

    this->static_pressure_measured = rocket.air_pressure + this->pressure_variance(this->generator);
}

void Sensors::kalman_filter(const SingleStageRocket& rocket, double dt) {
    Eigen::MatrixXd F(4,4);
}

void Sensors::my_filter(const SingleStageRocket& rocket, double dt) {
    Eigen::MatrixXd F(4,4);
}

void Sensors::get_computed_quantities(const SingleStageRocket& rocket, double time) {
    this->get_measured_quantities(rocket);

    double dt = time - this->time_old;

    this->kalman_filter(rocket,dt);

    this->time_old = time;
}
