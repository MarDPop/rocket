
#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include "../include/thruster.h"

/*
    rt - throat radius in m
    m0 - fuel mass kg
    rho - fuel density kg/ m3
    h - heating value kJ/kg
    k - specific heat ratio
    Ab - initial burn area
    v0 - initial volume
    P0
    t0
*/
void simulation::compute_chamber_pressure_simple(double rt, double m0, double k, double Ab, double dt, const std::string& fn ) {
    ofstream outputfile;
    outputfile.open (fn);

    double A_star = M_PI*rt*rt;


    myfile.close();
}
