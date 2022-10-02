#pragma once

#include "thruster.h"
#include <memory>
#include <vector>

class Vehicle_1D {

public:

    double ground_pressure = 100000;

    double ground_temperature = 297;

    double mass = 10;

    std::unique_ptr< Thruster1D > thruster;

    double CD0 = 0.7;

    double Aref = 0.01;

    double recording_interval = 1;

    double time_step = 0.01;

    std::vector<double> heights;

    Vehicle_1D();
    virtual ~Vehicle_1D();

    void launch();

};
