#pragma once

#include <vector>

#include "Gravity.h"
#include "Atmosphere.h"
#include "Geometry.h"

struct Environment
{

    Gravity* gravity = nullptr;

    Atmosphere* atmosphere = nullptr;

    Geometry* geometry = nullptr;    

};