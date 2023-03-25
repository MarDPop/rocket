#pragma once

#include "Kinematics.h"

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

struct Commands
{
    Vector force;
    Vector torque;
    inline Commands() : force((char)0), torque((char)0) {}
};

//guidance should control things like chute activation
class Guidance
{
protected:

    Commands commands;

public:

    Guidance();
    virtual ~Guidance();

    virtual const Commands& get_commands(const KinematicState& estimated_state, double time);
};

class SimpleAscent : public virtual Guidance
{

public:

    Parachute* chute;

    SimpleAscent();

    Commands get_commands(const KinematicState& estimated_state, double time) override;
};
