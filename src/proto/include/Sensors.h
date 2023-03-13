#pragma once

#include "Atmosphere.h"

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

/*
struct Measurements
{

};

class SensorsBase
{

public:

    SensorsBase();
    ~SensorsBase();

};
*/

class SingleStageRocket;

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
    ~Sensors();

    inline double get_measured_dynamic_pressure() const
    {
        return this->measured.total_pressure - this->measured.static_pressure;
    }

    inline const measured_quantities& get_measured_quantities() const
    {
        return this->measured;
    }

    inline void set_delay(double delay)
    {
        this->delay = delay;
    }

    inline void calibrate(const altitude_cal& cal)
    {
        this->cal = cal;
        this->cal.exp_const = -AltitudeTable::R_AIR / cal.g;
        this->cal.inverse_ref_pressure = 1.0/cal.pressure;
    }

    void set_sensor_variances(double sigma_pressure, double sigma_temperature, double sigma_accelerometer, double sigma_gyro);

    void init();

    void update(const SingleStageRocket& rocket, double time);

};
