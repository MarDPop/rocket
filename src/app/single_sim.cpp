#include <iostream>
#include "../common/include/util.h"
#include "../common/include/Cartesian.h"
#include "../6DOF/include/SingleStageRocket.h"
#include <string>
#include <cstdio>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

void print_out(SingleStageRocket& rocket, const char* fn) {

    FILE* file = fopen(fn,"w");

    if(!file) {
        throw std::invalid_argument("could not open file.");
    }

    fprintf(file,"39.94426809919236 -104.94474985717818 1606.0\n");

    int nLines = rocket.record.position.size();

    for(int i = 0; i < nLines; i++) {
        const double* pos = rocket.record.position[i].data;
        const double* q = rocket.record.orientation[i].data;
        fprintf(file,"%7.2f % .6e % .6e % .6e % 8.6f % 8.6f % 8.6f % 8.6f % 8.6f % 8.6f % 8.6f % 8.6f % 8.6f\n",
                    i*rocket.record.t_interval, pos[0], pos[1], pos[2], q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8]);
    }

    fclose(file);
}

int main(int argc, char *argv[]) {

    if(argc < 3) {
        throw "need input and output file.";
    }
    std::cout << "Loading File: " + std::string(argv[1]) << std::endl;

    std::string fn = argv[1];
    SingleStageRocket rocket(fn);

    double dt = 1.0/256.0;
    if (argc > 3) {
        dt = std::stod(argv[3]);
    }

    std::cout << "Running Simulation." << std::endl;

    rocket.launch(dt);

    std::cout << "Printing." << std::endl;

    print_out(rocket,argv[2]);

    std::cout << "Done. Press enter to exit" << std::endl;

    char b;
    std::cin >>  b;

}
