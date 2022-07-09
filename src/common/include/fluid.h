#pragma once

struct isentropic_flow {
    double gamma;
};

struct Molecule;

template<int M>
struct Gas {
    double mixture[M];
    Molecule* species[M];

    double gamma;
    double cp;
    double cv;
    double molecular_weight;
    double R_specific;
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
