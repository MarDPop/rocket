#pragma once

#include "../../common/include/Cartesian.h"

#include <array>

using namespace Cartesian;

class Parachute
{
protected:

friend class SingleStageControl;

    std::array<double,2> area; // initial, final
    std::array<double,2> CD; // initial, final

    double time_old;
    double time_deployed;

    bool deployed = false;
    bool fully_deployed = false;

    void deploy(double time);

    SingleStageRocket& rocket;

public:

    Vector tether_force;

    Parachute(SingleStageRocket& _rocket);
    ~Parachute();

    inline void set_deployment_terms(std::array<double,2> _area, std::array<double,2> _CD)
    {
        this->area = _area;
        this->CD = _CD;
    }

    inline bool is_deployed()
    {
        return this->deployed;
    }

    void update(double time);

};
