#pragma once

#include "SingleStageRocket.h"
#include "Atmosphere.h"
#include <string>
#include <vector>
#include <memory>

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

struct Launch_Parameters
{
    double latitude = 39.94426809919236;
    double longitude = -104.94474985717818;
    double altitude = 1606.0;
    double heading = 0;
    double pitch_angle = 0.05;
};

class SingleStageSimulation
{
    SingleStageRocket rocket;

    std::unique_ptr<Atmosphere> atmosphere;

    Recording record;

    Launch_Parameters launch;

public:

    SingleStageSimulation();
    ~SingleStageSimulation();

    void load(std::string fn);

    void run(std::string fn, const bool debug = false);

};
