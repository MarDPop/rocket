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

    static std::array<double,6> kepler2cartesian(const std::array<double,7>& oe); // const position is mu

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

class Earth {

    static const double EARTH_MU = 3.986004418e14; // m3 s-2
    static const double EARTH_AVG_R = 6371000; // m
    static const double EARTH_AVG_D = 12742000; // m
    static const double EARTH_POLAR_R = 6356752.314; // m
    static const double EARTH_SIDEREAL = 86164.1; // m
    static const double EARTH_FLATTENING = 0.003352810664747;
    static const double EARTH_EQUATOR_R = 6378137; // m
    static const double EARTH_ROT = 7.29211505392569E-5; // rad/s

    static const double a1 = 42697.67270715754; //a1 = a*e2
    static const double a2 = 1.8230912546075456E9; //a2 = a1*a1
    static const double a3 = 142.91722289812412; //a3 = a1*e2/2
    static const double a4 = 4.557728136518864E9; //a4 = 2.5*a2
    static const double a5 = 42840.589930055656; //a5 = a1+a3
    static const double a6 = 0.9933056200098622; //a6 = 1-e2
    static const double e2 = 0.006694380066765; //WGS-84 first eccentricity squared
    static const double e2prime = 0.006739496819936; // second eccentricity

};
