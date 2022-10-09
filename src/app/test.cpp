#include <iostream>
#include "../1D/include/structures.h"
#include "../common/include/structures.h"
#include "../common/include/util.h"
#include "../common/include/RocketShape.h"
#include "../common/include/Cartesian.h"
#include "../1D/include/thruster.h"
#include "../1D/include/Vehicle.h"
#include "../6DOF/include/Vehicle.h"
#include "../6DOF/include/Stage.h"
#include "Simulation.h"
#include <memory>
#include <cmath>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

void test_rocket_shape() {
    RocketShape rshape;
    rshape.nozzle_shape = 2;
    Curve out = rshape.generate_constant_dx(0.001);

    Cartesian::Vector v;

    util::print_table("test/test.txt",out.x,out.y );
}

void test_sugar() {
    SugarThruster thruster;

    thruster.fuel = std::make_unique<Fuel_KNSU>();

    double r_outer = 0.025;
    double r_inner = 0.01;
    double len = 0.15;

    double V = M_PI*r_outer*r_outer*len;
    double A = M_PI*2*r_inner*len;
    double M = (V - M_PI*r_inner*r_inner*len)*thruster.fuel->density;

    double At = M_PI*r_inner*r_inner;
    double Ae = At*3;
    thruster.set_areas(At,Ae);

    thruster.compute(A,V,M,5e-4);

    thruster.save("test_thruster.txt");


}

void test() {


}

int main_test(int argc, char *argv[]) {

    test();

    return 1;
}
