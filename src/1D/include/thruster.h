#pragma once

#include <fuctional>
#include <vector>

class Thruster {

    double thrust;

    double mass_rate;

public:

    inline double get_thrust() {
        return this->thrust;
    }

    inline double get_mass_rate() {
        return this->mass_rate;
    }

    Thruster() {}
    virtual ~Thruster() {}

    virtual void update(double ambient_pressure, double time, double throttle);

}

struct Fuel {
    enum Fuel { KNSU, KNSB, KNEY, KNDX, KNMN}; // KNSU - SUCROSE , KNSB - SORBITOL, KNEY - EURYTHTROL, KNDX - DEXTROSE, KNMN - Mannitol
    double density;

    double burn_rate(double pressure, double temperature);

    static double saint_roberts_burn(double a, double n, double r0, double P) {
        return a*pow(P,n) + r0;
    }

};

class Nozzle {

    double A_star;

    double area_ratio;

    double P_total;

    double T_total;

public:

    Nozzle();

    static inline double chocked_flow_mass_flux(double p_total, double t_total, double k, double mw) {
        double k1 = k+1;
        return p_total*sqrt(k*mw/(t_total*physics::R_GAS*pow(0.5*k1,k1/(k-1))));
    }

    static inline double isentropic_exit_mach(double area_ratio, double k) {
        double k1 = k - 1;
        double k2 = k + 1;

        area_ratio = pow(area_ratio, 2*k1/k2)*k2*0.5;

        M_guess = 3*(area_ratio - 0.5);

        k1 *= 0.5;

        for(int iter = 0; iter < 10; iter++) {

        }

    }

    double get_mass_rate() {

    }

    double get_exit_velocity(double ambient_pressure) {
        double M_exit = Nozzle::isentropic_exit_mach(this->A)
    }



};

class SolidThruster : public Thruster {

    std::vector<double> time;

    std::vector<double> pressure_chamber;

    std::vector<double> temperature_chamber;

public:

    SolidThruster();
    virtual ~SolidThruster();

    virtual void update(double ambient_pressure, double time, double throttle);

    void save(std::string fn);

    void load(std::string fn);

}

class SugarThruster : public SolidThruster {



public:

    std::unique_ptr<Fuel> fuel;

    public SugarThruster();





}
