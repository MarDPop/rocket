#include <iostream>
#include "../common/include/util.h"
#include "../common/include/Cartesian.h"
#include "../proto/include/SingleStageSimulation.h"
#include <string>
#include <cstdio>
#include <exception>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

int main(int argc, char *argv[]) {

    if(argc < 3) {
        throw std::invalid_argument("need input and output file.");
    }
    std::cout << "Loading File: " + std::string(argv[1]) << std::endl;

    SingleStageSimulation sim;

    sim.load(argv[1]);

    sim.run(argv[2]);

    std::cout << "Done. Press enter to exit" << std::endl;

    char b;
    std::cin >>  b;

}
