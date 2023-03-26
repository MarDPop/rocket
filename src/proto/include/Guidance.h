#pragma once

#include "Kinematics.h"

#include <unordered_map>
#include <string>

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

struct Commands
{
    Vector angular_velocity_in_body;
    std::unordered_map<std::string, double> other_values;
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

    Parachute* chute = nullptr;

    SimpleAscent();

    Commands get_commands(const KinematicState& estimated_state, double time) override;
};

class VerticalAscent : public virtual Guidance
{
    double proportionalVelocityConstant = 0.0;

    double proportionalOrientationConstant = 0.0;

public:

    Parachute* chute = nullptr;

    VerticalAscent();

    inline void setProportionalConstants(double velocityK, double orientationK)
    {
        this->proportionalVelocityConstant = velocityK;
        this->proportionalOrientationConstant = orientationK;
    }

    Commands get_commands(const KinematicState& estimated_state, double time) override;
};
