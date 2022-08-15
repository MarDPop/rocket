#pragma once

#include <fuctional>
#include <vector>

class Thruster {

    double pressure_chamber;

    double temperature_chamber;

    double area_ratio;

public:

    double thrust;
    double mass_rate;

    Thruster() {}
    virtual ~Thruster() {}

    virtual void update(double ambient_pressure, double time, double throttle);

}

struct Fuel {
    enum Fuel { KNSU, KNSB, KNEY, KNDX, KNMN}; // KNSU - SUCROSE , KNSB - SORBITOL, KNEY - EURYTHTROL, KNDX - DEXTROSE, KNMN - Mannitol
    double density;

    std::function<double(double, double)> burn_rate;

    static double saint_roberts_burn(double a, double n, double r0, double P) {
        return a*pow(P,n) + r0;
    }

}

class SugarThruster : public Thruster {

    std::vector<double> time;
    std::vector<double> pressure_chamber;
    std::vector<double> temperature_chamber;

    Fuel fuel;

public:

    public SugarThruster

    void set_fuel(const Fuel& fuel) {
        this->fuel = fuel;
    }



}
