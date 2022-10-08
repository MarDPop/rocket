#pragma once

#include "Component.h"
#include "Action.h"

#include <vector>
#include <string>

class Thruster : public virtual Action ,  public virtual Component {

    using Action::center;

    double thrust;

    double mass_rate;

public:

    inline Thruster() {
        this->force.zero();
        this->center.zero();
        this->moment.zero();
        this->mass = 0;
    }
    inline virtual ~Thruster() {}

    inline void set_thrust(double thrust) {
        this->thrust = thrust;
        this->force.x(thrust);
    }

    inline void set_mass_rate(double mass_rate) {
        this->mass_rate = mass_rate;
    }

    inline double get_thrust() const {
        return this->thrust;
    }

    inline double get_mass_rate() const {
        return this->mass_rate;
    }

    virtual void update(double time) override;

};

class SolidThruster_precomputed : public virtual Thruster {

    std::vector<double> times;

    std::vector<double> mass_rates;

    std::vector<double> p_exit;

    std::vector<double> v_exit;

public:


    SolidThruster_precomputed(const std::string& fn);
    ~SolidThruster_precomputed();

    void update(double time) override;
};

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
