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
