#pragma once

#include "Cartesian.h"
#include <cstring>

using namespace Cartesian;

union Geodetic {
    double v[3];
    struct {
        double latitude;
        double longitude;
        double altitude;
    };

    void operator=(const Geodetic* lla) {
        memcpy(&lla,this,3*sizeof(double));
    }
};

struct Ellipsoid {

    double polar_radius;

    double equatorial_radius;

    virtual double get_radius_by_latitude(double latitude) = 0;

    virtual double get_radius_by_z(double z) = 0;

    static double vincentyFormulae(double long1, double lat1, double long2, double lat2);

    static Vector geodetic2ecef(const Geodetic& lla);

    static Geodetic ecef2geodetic(const Vector& ecef);

};

class Geoid {

public:

    virtual double get_geoid_height(const double latitude, const double longitude);

    virtual double get_geoid_height(const double* ecef);

};

template <int N, int M>
class EGM : public Geoid {

public:

    double C[N][M];

    double S[N][M];

    double P[N][M];

    double J[N];

};

class EGM84 : public EGM<180,180> {

public:

};


class EGM96 : public EGM<360,360> {


};

class EGM2008 : public EGM<2160,2160> {



};


class Orthometric {


};

