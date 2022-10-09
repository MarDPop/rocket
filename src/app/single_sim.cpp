#include <iostream>
#include "../common/include/util.h"
#include "../common/include/Cartesian.h"
#include "../6DOF/include/Vehicle.h"
#include "../6DOF/include/Stage.h"
#include "Simulation.h"
#include <memory>
#include <cmath>
#include <string>
#include <fstream>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

void print_out(SingleStageRocket& rocket, const char* fn) {

    std::ofstream file(fn);

    if(!file.is_open()) {
        throw std::invalid_argument("could not open file.");
    }

    int nLines = rocket.record.position.size();

    for(int i = 0; i < nLines; i++) {
        file << i*rocket.record.t_interval << " ";
        for(int j = 0; j < 3; j++) {
            file << rocket.record.position[i].data[j] << " ";
        }
        for(int j = 0; j < 9; j++) {
            file << rocket.record.orientation[i].data[j] << " ";
        }
        file << std::endl;
    }


}

int main(int argc, char *argv[]) {

    if(argc < 3) {
        throw "need input and output file.";
    }
    std::cout << "Loading File: " + std::string(argv[1]) << std::endl;

    std::string fn = argv[1];
    SingleStageRocket rocket(fn);

    double dt = 0.1;
    if (argc > 3) {
        dt = std::stod(argv[3]);
    }

    rocket.launch(dt);

    std::cout << "Done. Printing." << std::endl;

    print_out(rocket,argv[2]);
}
