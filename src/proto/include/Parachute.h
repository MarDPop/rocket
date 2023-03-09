#pragma once

#include "../../common/include/Cartesian.h"

#include <array>

using namespace Cartesian;

class Parachute
{
protected:

friend class SingleStageControl;

    /**
    * drag coefficient multiplied by area
    */
    double CDA = 1.0;

    double time_old;

    double time_deployed;

    bool deployed = false;

    void deploy(double time);

    SingleStageRocket& rocket;

public:

    Vector tether_force;

    Parachute(SingleStageRocket& _rocket);
    Parachute(SingleStageRocket& _rocket, double _CDA);
    ~Parachute();

    inline bool is_deployed()
    {
        return this->deployed;
    }

    virtual void update(double time);

};

class ConstrainedParachute : public virtual Parachute
{

};
