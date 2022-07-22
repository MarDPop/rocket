#include <iostream>
#include "../1D/include/structures.h"
#include "../common/include/structures.h"
#include "../common/include/util.h"
#include "../common/include/RocketShape.h"
#include "../common/include/Cartesian.h"

int main(int argc, char *argv[]) {

    RocketShape rshape;
    rshape.nozzle_shape = 2;
    Curve out = rshape.generate_constant_dx(0.001);

    Cartesian::Vector v;

    util::print_table("test/test.txt",out.x,out.y );
}
