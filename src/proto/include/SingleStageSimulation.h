#pragma once

#include "SingleStageRocket.h"
#include <string>
#include <vector>

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

/**
* Internal struct definition to record states
*/
struct Recording
{
    /**
    * Time interval between recordings (sec)
    */
    double t_interval = 0.25;

    /**
    * Position (m)
    */
    std::vector<Vector> position;

    /**
    * Full CS orientation
    */
    std::vector<Axis> orientation;

    /**
    * Mass (kg)
    */
    std::vector<double> mass;

    /**
    * Any other test value we want
    */
    std::vector<double> test_value;
};

class SingleStageSimulation
{

    SingleStageRocket rocket;

    Recording record;

    double launch_angle = 0.02;

    double launch_heading = 0;

    FILE* output;

public:

    SingleStageSimulation();
    ~SingleStageSimulation();

    void load(std::string fn);

    void run(std::string fn, const bool debug = false);

};
