#pragma once

#include "Geodesy.h"
#include "Cartesian.h"
#include <memory>
#include <vector>

using namespace Cartesian;

class Ephemeris {

public:

    enum EPOCH {
        J2000, ICRF
    };

    enum FRAME {
        ICRS, GCRS, ECI
    };

    EPOCH epoch = J2000;

    FRAME frame = ICRS;

    virtual Vector get_position(double time) = 0;

    virtual Axis get_axis(double time) = 0;

    static double trueAnomalyFromEccentricAnomaly(const double EA, const double eccentricity);

    static double trueAnomalyFromMeanAnomaly(const double MA, const double eccentricity);

    static double trueAnomalyFromMeanAnomalyApprox(const double MA, const double eccentricity);

    static double meanAnomalyFromTrueAnomalyApprox(const double f, const double eccentricity);

    static double eccentricAnomalyFromMeanAnomaly(const double MA, const double eccentricity);

    static double meanAnomalyFromEccentricAnomaly(const double EA, const double eccentricity);

    static double eccentricAnomalyFromTrueAnomaly(const double TA, const double eccentricity);

    static double meanAnomalyFromTrueAnomaly(const double f, const double eccentricity);

    static std::array<double,6> kepler2cartesian(const std::array<double,7>& oe); // final position is mu

    static std::array<double,3> kepler2position(const std::array<double,6>& oe);

    static std::array<double,6> cartesian2kepler(const std::array<double,7>& state);

};

class Ephemeris_Standard : public Ephemeris {

    std::vector< double > times;

    std::vector< Vector > position;

    std::vector< Vector > delta;

public:

    Ephemeris_Standard();
    Ephemeris_Standard(const char* fn);
};


struct Planet {

    std::unique_ptr< Ellipsoid > ellipsoid;

    std::unique_ptr< Geoid > geoid;

    std::unique_ptr< Orthometric > orthometric;

    std::unique_ptr< Ephemeris > ephemeris;

};
