#pragma once

union Geodetic {
    double v[3];
    struct {
        double latitude;
        double longitude;
        double altitude;
    };
};

struct Ellipsoid {


};
