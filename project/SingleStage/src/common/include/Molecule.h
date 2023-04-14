#pragma once

#include <string>
#include <vector>

struct Element {
    unsigned char atomic_number;
    char symbol[2];
    unsigned char valence_electrons;
    double atomic_mass;
    double ionization_energy[8];
};

struct Periodic_Table {
    Element elements[200];
};

struct ThermalData {
    std::vector<double> temperature;
    std::vector<double> enthalpy;
    std::vector<double> entropy;
    std::vector<double> heat_capacity; // cp J/mol K
    std::vector<double> viscosity;
    double heat_formation_gas;
    double heat_formation_liquid;
};

struct PhaseData {
    std::vector<double> temperature;
    std::vector<double> vapor_pressure;
    double critical_pressure;
    double critical_temperature;
    double triple_pressure;
    double triple_temperature;
};

struct SpectrumData {
    std::vector<double> wave_length;
};

struct Molecule {

    ThermalData thermal;
    PhaseData phase;
    SpectrumData spectrum;

    std::string name;
    std::vector<int> elements;
    std::vector<int> number;
    double molecular_weight;

    double heat_of_formation;

    void load(std::string fn);
};

struct Reaction {

    Molecule Reactants[2];
    Molecule Products[2];

};
