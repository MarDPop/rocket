#pragma once

#include "Action.h"

struct Vehicle;

class GNC : public Action {

    Vehicle* vehicle;

public:

    GNC(Vehicle* v) : vehicle(v) {}
    ~GNC(){}

};
