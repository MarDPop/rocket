#pragma once

#include "Ellipsoid.h"
#include "Geoid.h"
#include "Orthometric.h"
#include <memory>

struct Planet {

    std::unique_ptr< Ellipsoid > ellipsoid;

    std::unique_ptr< Geoid > geoid;

    std::unique_ptr< Orthometric > orthometric;

};
