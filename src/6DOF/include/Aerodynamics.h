#pragma once

#include "Action.h"
#include "Atmosphere.h"

class Vehicle;

class Aerodynamics : public virtual Action {
protected:

    Vehicle* vehicle;

    Air* air;

public:

    Aerodynamics() {}

    void set_vehicle(Vehicle* vehicle);

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
