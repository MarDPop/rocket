#pragma once

#include <fuctional>
#include <vector>
#include <cmath>

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

};

struct Fuel {

    static constexpr double KNSU_DENSITY = 1.8;
     // KNSU - SUCROSE , KNSB - SORBITOL, KNEY - EURYTHTROL, KNDX - DEXTROSE, KNMN - Mannitol
    const double density;

    inline Fuel(double d) : density(d) {}

    virtual double burn_rate(double pressure, double temperature);

    static double saint_roberts_burn(double a, double n, double r0, double P) {
        return a*pow(P,n) + r0;
    }

};

struct Fuel_KNSU : Fuel {

    inline Fuel_KNSU() : Fuel(Fuel::KNSU_DENSITY) {}

    /*
        pressure in Pa
        temperature in K
    */
    inline double burn_rate(double pressure, double temperature) {
        if(pressure > 4.1e6) {
            return 9.12e-3 + 8e-10*pressure;
        }
        if(pressure > 7e5) {
            return 7.09e-3 + 1.29e-9*pressure;
        }
        return 3.5e-3 + 6.43e-9*pressure;
    }
};

struct Gas {
    double P_total;
    double T_total;
    double cp;
    double cv;
    double gamma;
    double mass_rate;
    double mw;
    double R_gas;
};

class Combustor {

    double Area_throat;

public:

    double P_total;

    double T_total;

    double cp;

    double cv;

    double gamma;

    double mass_rate;

    double mw;

    double R_gas;

    Combustor();

    virtual void update(double throttle, double time);
}

class Nozzle {

    double nozzle_efficiency;

    double area_exit;

    double area_ratio;

    std::unique_ptr<Combustor> combustor;

public:

    Nozzle();

    inline void set_exit_area(double area) {
        this->area_exit = area;
        if(this->combustor) {
            this->area_ratio = area/this->combustor->area_throat;
        }
    }

    inline void set_combustor(std::unique_ptr<Combuster> combustor) {
        this->combustor = std::move(combustor);
        this->area_ratio = area/this->combustor->area_throat;
    }

    inline void set_nozzle_efficiency(double nozzle_efficiency) {
        this->nozzle_efficiency = nozzle_efficiency;
    }

    static inline double chocked_flow_mass_flux(double p_total, double t_total, double k, double mw) {
        double k1 = k+1;
        return p_total*sqrt(k*mw/(t_total*physics::R_GAS*pow(0.5*k1,k1/(k-1))));
    }

    static double isentropic_exit_mach(double area_ratio, double k);

    static double isentropic_exit_mach(double area_ratio, double k, double M_guess);

    double get_thrust(double ambient_pressure);

};

class SolidThruster : public Thruster {

    std::vector<double> time;

    std::vector<double> pressure_chamber;

    std::vector<double> temperature_chamber;

    double area_ratio;

public:

    SolidThruster();
    virtual ~SolidThruster();

    inline void set_area_ratio(double area_ratio) {
        this->area_ratio = area_ratio;
    }

    virtual void update(double ambient_pressure, double time, double throttle);

    void save(std::string fn);

    void load(std::string fn);

}

class SugarThruster : public SolidThruster {



public:

    std::unique_ptr<Fuel> fuel;

    public SugarThruster();





}
