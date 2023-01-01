#pragma once

#include <vector>

class SingleStageRocket;

class SingleStageThruster {

    std::vector<double> pressures;

    std::vector<double> thrusts;

    std::vector<double> mass_rates;

    int idx = 0;

    int idx_final = 0;

    bool is_constant = true;

    double dT;

    double dM;

public:

    double thrust = 1000;

    double mass_rate = 1;

    void add_thrust_point(double pressure, double thrust, double mass_rate);

    void reset();

    void set(double pressure);

};
