#pragma once

#include "../../common/include/Cartesian.h"

#include "Action.h"

#include <array>

class SingleStageRocket;

using namespace Cartesian;

//TODO: rename to "Recovery"
class Parachute
{
protected:

    SingleStageRocket& _rocket;

    /**
    * drag coefficient multiplied by area
    */
    double _CDA = 1.0;

    double _time_old;

    double _time_deployed;

    bool _deployed = false;

    BodyAction _action;

public:

    Parachute(SingleStageRocket& rocket);
    Parachute(SingleStageRocket& rocket, double CDA);
    ~Parachute();

    inline void reset()
    {
        this->_deployed = false;
    }

    inline bool is_deployed()
    {
        return this->_deployed;
    }

    inline void deploy(double time)
    {
        this->_deployed = true;
        this->_time_deployed = time;
    }

    virtual const BodyAction& update(double time);

};

class ParachuteTimed : public virtual Parachute
{
    double _deployment_duration;

public:

    ParachuteTimed(SingleStageRocket& rocket, double CDA, double deployment_time);

    const BodyAction& update(double time);
};

class ConstrainedParachute : public virtual Parachute
{
public:

};
