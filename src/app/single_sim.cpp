#include <iostream>
#include "../common/include/util.h"
#include "../common/include/Cartesian.h"
#include "../proto/include/SingleStageSimulation.h"
#include <string>
#include <cstdio>
#include <exception>

#include <float.h>

int main(int argc, char *argv[]) {

    _clearfp();
    unsigned current_word = 0;
    const unsigned int SERIOUS_FP_EXCEPTIONS = _EM_DENORMAL | _EM_ZERODIVIDE | _EM_INVALID;
    _controlfp_s(&current_word, ~SERIOUS_FP_EXCEPTIONS , _MCW_EM);

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
