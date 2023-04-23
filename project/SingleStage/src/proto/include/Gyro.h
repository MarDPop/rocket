#pragma once

#include <random>

class Gyro
{

    std::default_random_engine generator;

    std::normal_distribution<double> variance;

public:

    Gyro();
    virtual ~Gyro();


};
