#include "../include/Sensors.h"

#include "../include/SingleStageRocket.h"

#include "../../../lib/Eigen/Dense"

#include <cmath>

void computed_quatities::set(const measured_quantities& measured, const altitude_cal& cal) {
    double pressure_ratio = measured.total_pressure > measured.static_pressure ? measured.total_pressure/measured.static_pressure : 1.0;

    this->mach_squared = 5*(pow(pressure_ratio, 2/7) - 1.0);

    this->airspeed = sqrt(this->mach_squared/(Atmosphere::AIR_CONST*measured.temperature));

    pressure_ratio = measured.static_pressure * cal.inverse_ref_pressure;

    double lapse_rate_measured = measured.temperature - cal.temperature;

    double height;
    if(fabs(lapse_rate_measured) < 1e-6) {
        height = log(pressure_ratio) * cal.temperature * cal.exp_const;
    } else {
        double inv_exp = lapse_rate_measured * cal.exp_const;
        double rhs = pow(pressure_ratio, inv_exp);
        height = (rhs - 1.0) * cal.temperature / lapse_rate_measured;
    }

    this->altitude = cal.altitude + height;
}

Sensors::Sensors() : generator(std::random_device{}()) {
    this->set_sensor_variances(0.5,0.1,0.01,0.00000001);
}

Sensors::~Sensors() {}

void Sensors::set_sensor_variances(double sigma_pressure, double sigma_temperature, double sigma_accel, double sigma_gyro) {
    this->barometer_variance = std::normal_distribution<double>(0,sigma_pressure);
    this->thermometer_variance = std::normal_distribution<double>(0,sigma_temperature);
    this->accelerometer_variance = std::normal_distribution<double>(0,sigma_accel);
    this->gyro_variance = std::normal_distribution<double>(0,sigma_gyro);
}

void Sensors::measure_quantities(const SingleStageRocket& rocket) {
    // add delay
    const auto& rocket_state = rocket.get_state();
    this->measured.angular_velocity = rocket_state.CS*rocket_state.angular_velocity;
    this->measured.acceleration = rocket_state.CS*rocket_state.acceleration;

    for(int i = 0; i < 3; i++)
    {
        this->measured.angular_velocity.data[i] += this->gyro_variance(this->generator);
        this->measured.acceleration.data[i] += this->accelerometer_variance(this->generator);
    }

    this->measured.total_pressure = rocket.get_atmosphere().values.pressure + rocket.get_aerodynamics().get_aero_values().dynamic_pressure + this->barometer_variance(this->generator);
    this->measured.static_pressure = rocket.get_atmosphere().values.pressure + this->barometer_variance(this->generator);
    this->measured.temperature = rocket.get_atmosphere().values.temperature + this->thermometer_variance(this->generator);
}

void Sensors::update(const SingleStageRocket& rocket, double time) {
    this->measure_quantities(rocket);

    this->computed.set(this->measured,this->cal);
}
