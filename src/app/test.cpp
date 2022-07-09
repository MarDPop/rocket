#include <iostream>
#include "../1D/include/structures.h"
#include "../common/include/structures.h"
#include "../common/include/util.h"
#include "../common/include/RocketShape.h"

int main(int argc, char *argv[]) {

    RocketShape rshape;
    rshape.nozzle_shape = 2;
    Curve out = rshape.generate_constant_dx(0.001);

    util::print_table("test.txt",out.x,out.y );
}
