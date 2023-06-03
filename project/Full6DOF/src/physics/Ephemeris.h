#pragma once

#include <array>
#include <vector>

struct Ephemeris
{
    union 
    {
        std::array<double,6> elements;
        struct 
        {
            double semi_major_axis;
            double eccentricity;
            double inclination;
            double longitude_of_ascending_node;
            double argument_of_periapse;
            double true_anomaly;
        };
    };

    const double MU;

    Ephemeris(  const std::array<double,6>& _elements, 
                double _MU) : 
                    elements(_elements), 
                    MU(_MU)
                    {}

    Ephemeris(double _MU) : MU(_MU) {}

    static std::array<double,6> elements2cartesian(const std::array<double,6>& elements) {}

    static std::array<double,6> cartesian2elements(const std::array<double,6>& cartesian) {}

};


class EphemerisHistory 
{
    std::vector<Ephemeris> _ephemeris;

    std::vector<double> _jd;

public:



};