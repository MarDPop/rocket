#pragma once

#include "Kinematics.h"
#include <vector>
#include <array>

#include <string>
#include "Action.h"
#include "Environment.h"
#include "../../common/include/Table.h"

//TODO: rename to propulsion
class Thruster
{
protected:

    double _time_old = 0.0;

    /**
    * thrust value in N
    */
    double _thrust = 0.0;

    /**
    * mass rate in kg/s
    */
    double _mass_rate = 0.0;

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
        this->_thrust = thrust;
        this->_mass_rate = thrust/(isp*9.806);
    }

    inline bool is_active()
    {
        return this->active;
    }

    inline double get_thrust()
    {
        return this->_thrust;
    }

    inline double get_mass_rate()
    {
        return this->_mass_rate;
    }

    inline const Inertia_Basic& get_inertia()
    {
        return this->inertia_fuel;
    }

    inline void set_fuel_inertia(const Inertia_Basic& inertia_fuel)
    {
        this->inertia_fuel = inertia_fuel;
    }

    inline void set_center_of_mass(const double& Z)
    {
        this->inertia_fuel.CoM_axial = Z;
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

class EstesThruster : public virtual Thruster
{
    std::string _name;

    IncrementingTable _table;

    double _propellant_mass;

    double _total_mass;

    double _total_impulse;

    double _chute_delay;

    double _burnout_time;

    double _inv_avg_exit_velocity;

    int _length_mm;

    int _diameter_mm;

    void update_thrust_and_massrate(double time) override;

public:

    EstesThruster(const Environment& atmosphere);

    inline double get_burnout_time()
    {
        return this->_burnout_time;
    }

    inline void set_chute_delay(double delay)
    {
        this->_chute_delay = delay;
    }

    inline double get_chute_delay()
    {
        return this->_chute_delay;
    }

    inline double get_propellant_mass()
    {
        return this->_propellant_mass;
    }

    void generate(char Class, int AvgThrust, int Delay);

    void load(std::string fn) override;

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

    inline void reset()
    {
        this->idx = 0;
    }

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
