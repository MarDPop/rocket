#pragma once

#include "../physics/Body.h"

#include "Action.h"
#include <memory>

class Vehicle_3DOF : public virtual Body_Point_Mass
{

    std::vector<Eigen::Vector3d*> _forces;

public:

    inline Vehicle_3DOF(){}

};


template<typename T, unsigned N_ACTIONS>
class Vehicle : public virtual T
{

    std::array<Action*,N_ACTIONS> _actions;

public:

    inline Vehicle(){}

};

