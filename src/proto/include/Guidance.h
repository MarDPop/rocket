#pragma once

#include "Kinematics.h"

struct Commands
{
    KinematicState* state = nullptr;
};

//guidance should control things like chute activation
class Guidance
{

public:

    Guidance();
    virtual ~Guidance();

    virtual KinematicState get_commanded_state(const KinematicState& estimated_state, double time);
};

class SimpleAscent : public virtual Guidance
{
    KinematicState state;

public:

    Parachute* chute;

    SimpleAscent();

    KinematicState get_commanded_state(const KinematicState& estimated_state, double time) override;
};
