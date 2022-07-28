#pragma once

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

struct Action {

    Vector force;
    Vector moment;
    Vector location;

    virtual void update(const double& time);

}
