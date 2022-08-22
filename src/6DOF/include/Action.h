#pragma once

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

struct Action {

    Vector force;
    Vector moment;
    Vector center;

    virtual void update(double time) = 0;

};
