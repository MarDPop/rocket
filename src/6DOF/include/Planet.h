#pragma once

#include "Atmosphere.h"
#include "Gravity.h"
#include "Geoid.h"
#include <memory>

class Planet {

public:

    std::unique_ptr< Atmosphere > atmosphere;

    std::unique_ptr< Gravity > gravity;

    std::unique_ptr< Geoid > geoid;

};
