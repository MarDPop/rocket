#pragma once

#include "SingleStageRocket.h"
#include <string>

class SingleStageSimulation {

    SingleStageRocket rocket;

    double launch_angle = 0;

    double launch_heading = 0;

public:

    SingleStageSimulation();
    ~SingleStageSimulation();

    void load(std::string fn);

    void run(const char* fn);

};
