#pragma once

#include "Environment.h"

#include <random>
#include <memory>

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

struct altitude_cal
{
    double altitude;
    double g;
    double temperature;
    double pressure;
    double inverse_ref_pressure;
    double exp_const;
    double accelerometer_cog_distance;
};

struct measured_quantities
{
    double static_pressure;
    double total_pressure;
    double temperature;
    Vector angular_velocity;
    Vector acceleration;
};

struct computed_quatities
{
    double mach_squared;
    double altitude;
    double airspeed;
    void set(const measured_quantities& measured, const altitude_cal& cal);
};

class SingleStageRocket;

class Sensor
{
    const SingleStageRocket& _rocket;

    std::default_random_engine _generator;

public:

    Sensor(const SingleStageRocket& rocket);
    virtual ~Sensor();

    virtual void update(double time) = 0;

};


class Sensors
{

    std::default_random_engine generator;

    std::normal_distribution<double> barometer_variance;

    std::normal_distribution<double> accelerometer_variance;

    std::normal_distribution<double> thermometer_variance;

    std::normal_distribution<double> gyro_variance;

    altitude_cal cal;

    measured_quantities measured;

    computed_quatities computed;

    double delay = 0.0;

    void measure_quantities(const SingleStageRocket& rocket);

public:

    Sensors();
    virtual ~Sensors();

    inline double get_measured_dynamic_pressure() const
    {
        return this->measured.total_pressure - this->measured.static_pressure;
    }

    inline const measured_quantities& get_measured_quantities() const
    {
        return this->measured;
    }

    inline const computed_quatities& get_computed_quantities() const
    {
        return this->computed;
    }

    inline void set_delay(double delay)
    {
        this->delay = delay;
    }

    inline void calibrate(double altitude, double g, double temperature, double pressure)
    {
        this->cal.altitude = altitude;
        this->cal.g = g;
        this->cal.temperature = temperature;
        this->cal.pressure = pressure;
        this->cal.accelerometer_cog_distance = 0.0;
        this->cal.exp_const = -Environment::R_AIR / g;
        this->cal.inverse_ref_pressure = 1.0/pressure;
    }

    void set_sensor_variances(double sigma_pressure, double sigma_temperature, double sigma_accelerometer, double sigma_gyro);

    void init();

    void update(const SingleStageRocket& rocket, double time);

};
