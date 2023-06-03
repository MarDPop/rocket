#pragma once

#include <memory>

#include "Gravity.h"
#include "Atmosphere.h"
#include "Ephemeris.h"
#include "Geometry.h"

struct Planet
{
    EphemerisHistory _ephemeris;

    Gravity& _gravity;

    Atmosphere& _atmosphere;

    Geometry& _geometry;

    Planet(     Gravity& gravity, 
                Atmosphere& atmosphere, 
                EphemerisHistory ephemeris, 
                Geometry& geometry) :
                    _gravity(gravity), 
                    _atmosphere(atmosphere), 
                    _ephemeris(std::move(ephemeris)), 
                    _geometry(geometry) {}

    void set_Julian_date(double jd) {}
};