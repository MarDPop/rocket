#pragma once

#include "Kinematics.h"

#include <unordered_map>
#include <string>

#include "../../common/include/Cartesian.h"

class Parachute;

using namespace Cartesian;

struct Commands
{
    Vector z_axis_inertial;
    std::unordered_map<std::string, double> other_values;
    std::unordered_map<std::string, bool> flags;
};

//guidance should control things like chute activation
class Guidance
{
protected:

    Commands _commands;

public:

    Parachute* chute = nullptr;

    Guidance();
    virtual ~Guidance();

    virtual const Commands& get_commands(const KinematicState& estimated_state, double time);
};

class GuidanceTimedParachute : public virtual Guidance
{
protected:

    const double _deploy;

public:

    GuidanceTimedParachute(double burnout, double delay);

    const Commands& get_commands(const KinematicState& estimated_state, double time) override;
};

class GuidanceSimpleAscent : public virtual Guidance
{

public:

    GuidanceSimpleAscent();

    const Commands& get_commands(const KinematicState& estimated_state, double time) override;
};

class GuidanceVerticalAscent : public virtual Guidance
{
    double proportional = 1.0;

    double damping = 0.1;

public:

    GuidanceVerticalAscent();

    inline void setProportionalConstants(double P, double D)
    {
        this->proportional = P;
        this->damping = D;
    }

    const Commands& get_commands(const KinematicState& estimated_state, double time) override;
};
