#include <iostream>
#include "../common/include/util.h"
#include "../common/include/Cartesian.h"
#include "../proto/include/SingleStageSimulation.h"
#include <string>
#include <cstdio>
#include <exception>

#include <float.h>

void run_tests()
{
    double angle = 0.1;
    double az = 0.5;
    double el = 0.5;
    Vector axis(cos(az)*cos(el),sin(az)*cos(el),sin(el));

    std::cout << angle << std::endl;
    for(int i = 0; i < 3; i++)
    {
        std::cout << axis.data[i] << " ";
    }
    std::cout << std::endl;

    Axis A(angle,axis);
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            std::cout << A.data[3*i + j] << " ";
        }
        std::cout << std::endl;
    }

}

int main(int argc, char *argv[]) {

    _clearfp();
    unsigned current_word = 0;
    const unsigned int SERIOUS_FP_EXCEPTIONS = _EM_DENORMAL | _EM_ZERODIVIDE | _EM_INVALID;
    _controlfp_s(&current_word, ~SERIOUS_FP_EXCEPTIONS , _MCW_EM);

    run_tests();

    if(argc < 3)
    {
        throw std::invalid_argument("need input and output file.");
    }

    bool run_debug = true;
    if(argc > 3)
    {
        run_debug = (argv[3][0] == '1' || argv[3][0] == 't');
    }
    std::cout << "Loading File: " + std::string(argv[1]) << std::endl;

    SingleStageSimulation sim;

    sim.load(argv[1]);

    sim.run(argv[2],run_debug);

    std::cout << "Done. Press enter to exit" << std::endl;

    char b;
    std::cin >>  b;

}
