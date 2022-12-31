#pragma once

#include "Molecule.h"
#include <cmath>

class flow {
    const double k;
    const double k1;
    const double k2;
    const double k3;
    const double k4;
    const double k5;

public:

    inline flow(double gamma) : k(gamma), k1(gamma-1), k2(gamma+1), k3(k1*0.5), k4(k2*0.5), k5(k4/k1) {}

    inline static double beta(double M, double k) {
        return 1 + 0.5*(k-1)*M*M;
    }

    inline static double p_static2p_total(double M, double k) {
        double k1 = k-1;
        return pow(1 + 0.5*k1*M*M,-k/k1);
    }

    inline static double p_dynamic(double p_static, double M, double k) {
        double k1 = k-1;
        return p_static*pow(0.5*k1*M*M,-k/k1);
    }

    inline static double rho_static2rho_total(double M, double k) {
        double k1 = k-1;
        return pow(1 + 0.5*k1*M*M,1.0/k1);
    }

    inline static double A_ratio(double M, double k) {
        double tmp = 0.5*(k+1);
        double k1 = k-1;
        return pow((1 + 0.5*k1*M*M)/tmp,tmp/k1)/M;
    }

    inline static double normal_shock_sqmach_ratio(double M2, double k) {
        double k1 = k-1;
        return (k1*M2 +2 )/(2*k*M2 - k1);
    }

    inline static double normal_shock_total_pressure_ratio(double M, double k) {
        double k1 = k - 1;
        double k2 = k + 1;
        double M2 = M*M;
        double a = pow((k2*M2)/(k1*M2+2),k);
        return pow((a*k2)/(2*k*M2 - k1),1.0/k1);
    }

    inline static double normal_shock_temperature_ratio(double M, double k) {
        double k1 = k - 1;
        double k2 = k + 1;
        double M2 = M*M;
        return (2*k*M2 - k1)*(k1*M2 + 2)/(k2*k2*M2);
    }

    inline static double normal_shock_static_pressure_ratio(double M, double k) {
        double M2 = M*M;
        return (k+1)*M2/((k-1)*M2 + 2);
    }
};

struct Gas {
    double gamma;
    double cp;
    double cv;
    double molecular_weight;
    double R_specific;
};

template<int M>
struct Gas_Mixture : Gas {
    Molecule* species[M];
    double mixture[M];
};

template<int N>
struct Fluid {
    double data[N];
    double* density;
    double* energy;
    double* momentum;

    double temperature;
    double kinetic_energy;
    Gas* gas = nullptr;
};

