#pragma once

#include "Action.h"
#include "Atmosphere.h"

class Aerodynamics : public virtual Action {

    Vehicle* vehicle;

    Air* air;

public:

    Aerodynamics() {}

    inline void set_vehicle(Vehicle* vehicle){
        this->vehicle = vehicle;
        this->air = vehicle->planet->atmosphere->air;
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

    void update(double time);

};
