#pragma once

#include "../../1D/include/thruster.h"
#include <vector>

class SingleStageThruster {

protected:

    double thrust;

    double mass_rate;

public:

    SingleStageThruster();
    SingleStageThruster(double t, double isp);
    ~SingleStageThruster();

    inline double get_thrust()
    {
        return this->thrust;
    }

    inline double get_mass_rate()
    {
        return this->mass_rate;
    }

    virtual void set(double pressure, double time);
}


class PressureThruster : public virtual SingleStageThruster {

    std::vector<double> pressures;

    std::vector<double> thrusts;

    std::vector<double> mass_rates;

    int idx = 0;

    int idx_final = 0;

    double dT;

    double dM;

public:

    void add_thrust_point(double pressure, double thrust, double mass_rate);

    void reset();

    void set(double pressure, double time) override;

};

class ComputedThruster : public virtual SingleStageThruster
{
    struct performance_values
    {
        double chamber_pressure;
        double ideal_exit_velocity;
        double mass_rate;
        double mass;
        double Ixx;
        double Izz;
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

    int _tidx;

    void set_nozzle_parameters(double frozen_gamma, double mw, double throat_area, double exit_area, double half_angle);

public:

    double mass;
    double Ixx;
    double Izz;

    ComputedThruster();
    ComputedThruster(std::string fn);
    ~ComputedThruster();

    void set(double pressure, double time) override;

    void generate(double A_b,
                  double n_exp,
                  double fuel_density,
                  double fuel_heating,
                  double gamma,
                  double mw,
                  double length,
                  double radius,
                  double bore_radius,
                  double throat_radius,
                  double exit_radius);

};
