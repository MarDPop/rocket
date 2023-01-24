#pragma once

#include <array>

class Fluid_Axisymmetric
{
    union
    {
        std::array<double,4> vals;
        struct
        {
            double density;
            double internal_energy;
            double momentum[2];
        };

    };

    double temperature;
    double pressure;

public:

};
