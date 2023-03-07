#pragma once

#include "Action.h"

struct Vehicle;

class GNC : public Action
{

    Vehicle* vehicle;

public:

    GNC();
    ~GNC();

    inline void set_vehicle(Vehicle* vehicle) {
        this->vehicle = vehicle;
    }

};
