#pragma once

#include "../physics/Body.h"

#include "GNC.h"
#include "Action.h"
#include "../physics/Stage.h"

#include <memory>

class Vehicle_3DOF : public virtual Body_Point_Mass
{

    std::vector<Eigen::Vector3d*> _forces;

public:

    inline Vehicle_3DOF(){}

};


template<class T, unsigned N_ACTIONS>
class Vehicle : public virtual T
{

    std::array<BodyAction*, N_ACTIONS> _actions;

    GNC _gnc;

    Environment _environment;

public:

    inline Vehicle(){}

};


