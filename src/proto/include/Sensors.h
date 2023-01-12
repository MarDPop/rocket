#pragma once

#include <random>
#include <memory>

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

struct altitude_cal {
    double altitude;
    double g;
    double temperature;
    double pressure;
    double exp_const;
    double accelerometer_cog_distance;
};

struct measured_quantities {
    double static_pressure;
    double total_pressure;
    double temperature;
    Vector angular_velocity;
    Vector acceleration;
};

struct computed_quatities {
    double mach;
    double altitude;
    void set(const measured_quantities& measured, const altitude_cal& cal);
};

struct rocket_state {
    Vector position;
    Vector velocity;
    Vector acceleration;
    Vector angular_velocity;
    Quaternion orientation;
};

class Sensors;

class Filter {

    double t_old;

    rocket_state state;

public:

    virtual void update(const Sensors& sensors, double t);

    inline const Vector& get_computed_angular_rate() {
        return this->state.angular_velocity;
    }

    inline const Vector& get_computed_position() {
        return this->state.position;
    }

    inline const Vector& get_computed_velocity() {
        return this->state.velocity;
    }

    inline const Vector& get_computed_acceleration() {
        return this->state.acceleration;
    }

    inline const Axis& get_computed_CS() {
        return this->state.orientation.to_rotation_matrix();
    }
};

class FilterMarius : public virtual Filter {

public:

    void update(const Sensors& sensors, double t) override;

};

class SingleStageRocket;

class Sensors {

    std::default_random_engine generator;

    std::normal_distribution<double> barometer_variance;

    std::normal_distribution<double> accelerometer_variance;

    std::normal_distribution<double> thermometer_variance;

    std::normal_distribution<double> gyro_variance;

    altitude_cal cal;

    measured_quantities measured;

    computed_quatities computed;

    double delay = 0.0;

public:

    std::unique_ptr<Filter> filter;

    Sensors();
    ~Sensors();

    inline double get_measured_dynamic_pressure() {
        return this->measured.total_pressure - this->measured.static_pressure;
    }

    inline void set_delay(double delay) {
        this->delay = delay;
    }

    inline void calibrate(const altitude_cal& cal) {
        this->cal = cal;
        this->cal.exp_const = -AltitudeTable::R_AIR / cal.g;
    }

    void set_sensor_variances(double sigma_pressure, double sigma_temperature, double sigma_accelerometer, double sigma_gyro);

    void init();

    void update(const SingleStageRocket& rocket, double time);

};
