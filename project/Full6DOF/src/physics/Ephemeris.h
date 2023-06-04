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

    static double trueAnomalyFromEccentricAnomaly(const double EA, const double eccentricity) {}

    static double trueAnomalyFromMeanAnomaly(const double MA, const double eccentricity) {}

    static double trueAnomalyFromMeanAnomalyApprox(const double MA, const double eccentricity) {}

    static double meanAnomalyFromTrueAnomalyApprox(const double f, const double eccentricity) {}

    static double eccentricAnomalyFromMeanAnomaly(const double MA, const double eccentricity) {}

    static double meanAnomalyFromEccentricAnomaly(const double EA, const double eccentricity) {}

    static double eccentricAnomalyFromTrueAnomaly(const double TA, const double eccentricity) {}

    static double meanAnomalyFromTrueAnomaly(const double f, const double eccentricity) {}

    static std::array<double,6> elements2cartesian(const std::array<double,6>& elements) {}

    static std::array<double,6> cartesian2elements(const std::array<double,6>& cartesian) {}

};


class EphemerisHistory 
{
    std::vector<Ephemeris> _ephemeris;

    std::vector<double> _jd;

public:



};