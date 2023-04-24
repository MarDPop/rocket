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
    Vector _relative_position;

    Vector _relative_velocity;

    double _deployment_duration;

    double _tether_length;

    double _tether_spring_constant;

    double _volume_deployed;

public:

    enum MATERIAL {DEFAULT_MATERIAL = 0, NYLON = 1};

    ConstrainedParachute(SingleStageRocket& rocket);

    inline void set_deployment_duration(double duration)
    {
        this->_deployment_duration = duration;
    }

    inline void set_tether_location(double Z)
    {
        this->_action.location.z = Z;
    }

    void set_params(double area, double tether_length, double youngs_modulus, double area_tether_cross_section);

    const BodyAction& update(double time);

};
