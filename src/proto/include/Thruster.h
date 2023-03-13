#pragma once

#include "Kinematics.h"
#include <vector>
#include <array>

class Thruster
{

protected:

    bool active = true;

    double thrust;

    double mass_rate;

    Inertia inertia;

public:

    SingleStageThruster();
    SingleStageThruster(double t, double isp);
    ~SingleStageThruster();

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

    inline const Inertia& get_inertia()
    {
        return this->inertia;
    }

    virtual void set(double pressure, double time) {}

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

    double fuel_mass;

    double time_old;

public:

    inline void set_fuel_mass(double mass)
    {
        this->fuel_mass = mass;
    }

    void add_thrust_point(double pressure, double thrust, double mass_rate);

    void reset();

    void set(double pressure, double time) override;

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

    double mass;
    double Ixx;
    double Izz;

    ComputedThruster();
    ~ComputedThruster();

    void load(std::string fn) override;

    void set(double pressure, double time) override;

    inline void reset_tidx()
    {
        this->_tidx = 0;
    }

    void generate(const generate_args& args, const double p_ambient = 100000); // TODO: find a way to set p_ambient

    void save(std::string fn);

};
