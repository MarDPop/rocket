#include "Guidance.h"

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

KinematicState Guidance::get_commanded_state(const KinematicState& estimated_state, double time);
{
    return estimated_state;
}

SimpleAscent::SimpleAscent()
{
    this->state.CS = Axis::eye();
    this->state.velocity.zero();
}

KinematicState SimpleAscent::get_commanded_state(const KinematicState& estimated_state, double time)
{
    if(!this->chute->is_deployed() && estimated_state.velocity.z < -1.0)
    {
        this->chute->deploy();
    }
    return this->state;
}
