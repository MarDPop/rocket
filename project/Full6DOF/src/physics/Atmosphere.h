#pragma once

#include "Coordinates.h"

struct Air
{
    double pressure = 101325; // Pa
    double temperature = 293.15; // K
    double inv_sound_speed = 0.00291346724; // s/m
    double density = 1.225; // kg/m3
    double dynamic_viscosity = 1.803e-7; // Pa s
};

struct Air_Extended
{

};

class Atmosphere
{

    Air& _air;

public:

    virtual void update(const Coordinate::Geodetic& lla, double time) {}

};