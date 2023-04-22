#pragma once

#include "Kinematics.h"
#include <vector>
#include <array>

#include <string>
#include "Action.h"
#include "Environment.h"

//TODO: rename to propulsion
class Thruster
{
protected:

    double time_old = 0.0;

    /**
    * thrust value in N
    */
    double thrust = 0.0;

    /**
    * mass rate in kg/s
    */
    double mass_rate = 0.0;

    const double& pressure;

    Inertia_Basic inertia_fuel;

    Vector thrust_vector;

    BodyAction action;

    bool active = true;

    virtual void update_inertia(double time);

    virtual void update_thrust_and_massrate(double time);

    virtual void update_thrust_vector(double time);

    void update_action();

public:

    Thruster(const Environment& atmosphere);
    ~Thruster();

    inline void set_performance(double thrust, double isp)
    {
        this->thrust = thrust;
        this->mass_rate = thrust/(isp*9.806);
    }

    inline bool is_active()
    {
        return this->active;
    }

    inline double get_thrust()
    {
        return this->thrust;
    }

    inline double get_mass_rate()
    {
        return this->mass_rate;
    }

    inline const Inertia_Basic& get_inertia()
    {
        return this->inertia_fuel;
    }

    inline void set_fuel_inertia(const Inertia_Basic& inertia_fuel)
    {
        this->inertia_fuel = inertia_fuel;
    }

    inline void set_thrust_location_in_body(const Vector& location)
    {
        this->action.location = location;
    }

    inline void set_thrust_vector(const Vector& thrust_vector)
    {
        this->thrust_vector = thrust_vector;
    }

    void set_time(double time);

    inline const BodyAction& get_action()
    {
        return this->action;
    }

    virtual void load(std::string fn);
};


class PressureThruster : public virtual Thruster {

    std::vector<double> pressures;

    std::vector<double> thrusts;

    std::vector<double> mass_rates;

    int idx = 0;

    int idx_final = 0;

    double dT;

    double dM;

    double time_old;

    void update_thrust_and_massrate(double time) override;

public:

    PressureThruster(const Environment& atmosphere);

    void add_thrust_point(double pressure, double thrust, double mass_rate);

    void reset();

    void load(std::string fn) override;

};

class ComputedThruster : public virtual Thruster
{
    struct performance_values
    {
        union
        {
            std::array<double,7> v;
            struct
            {
                double chamber_pressure;
                double chamber_temperature;
                double ideal_exit_velocity;
                double mass_rate;
                double mass;
                double Ixx;
                double Izz;
            };
        };


        performance_values(){}
        performance_values(double p, double t, double v, double r, double m, double x, double z) :
            chamber_pressure(p), chamber_temperature(t), ideal_exit_velocity(v), mass_rate(r), mass(m), Ixx(x) , Izz(z) {}
    };

    double _area_ratio;
    double _exit_area;
    double _half_angle;
    double _ideal_exit_mach;
    double _half_angle_factor;

    double _gamma;
    double _mw;
    double _pressure_ratio_critical;
    double _pressure_ratio_ideal;
    double _pressure_ratio_normal_exit;

    std::vector<double> _times;

    std::vector<performance_values> _values;

    std::vector<performance_values> _dvalues;

    unsigned _tidx = 0;

    unsigned _tidx_final = 0;

    void set_nozzle_parameters(double frozen_gamma, double mw, double throat_area, double exit_area, double half_angle);

    void load_precomputed(const std::vector<std::string>& lines);

    void load_parameter_set(const std::vector<std::string>& lines);

    void update_thrust_and_massrate(double time) override;

public:

    struct generate_args
    {
        double burn_coef;
        double burn_exp;
        double fuel_density;
        double fuel_heating;
        double gamma;
        double mw;
        double length;
        double radius;
        double bore_radius;
        double throat_radius;
        double exit_radius;
        double half_angle;
        double segment_gap;
        double combustion_efficiency;
        double discharge_coef;
        double chamber_efficiency;
        double nozzle_efficiency;
        int n_segments;
    };

    ComputedThruster(const Environment& atmosphere);
    ~ComputedThruster();

    void load(std::string fn) override;

    inline void reset_tidx()
    {
        this->_tidx = 0;
    }

    void generate(const generate_args& args, const double p_ambient = 100000); // TODO: find a way to set p_ambient

    void save(std::string fn);

};
