#pragma once

#include "Action.h"

class Aerodynamics : public Action {

    Air* air;

public:

    Aerodynamics() {}

    inline void set_air(Air* air) {
        this->air = air;
    }

};

class AerodynamicsBasic : public Aerodynamics {

    double CD0;

    double K;

    double CL_alpha;

    double CM_alpha;

    double stall_angle;

public:

    AerodynamicsBasic() {}

}
