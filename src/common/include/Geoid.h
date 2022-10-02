#pragma once

class Geoid {

public:

    virtual double get_geoid_height(const double& latitude, const double& longitude);

    virtual double get_geoid_height(const double& x, const double& y, const double& z);

};
