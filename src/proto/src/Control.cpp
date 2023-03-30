#include "Control.h"

void Control::get_outputs(const Commands& commands, const KinematicState& estimated_state, double time) {}

template<unsigned NUMBER_FINS>
Vector void FinControl<NUMBER_FINS>::get_desired_torque(const Commands& commands, const KinematicState& estimated_state)
{
    Vector delta_orientation = commands.axis_orientation - estimated_state.CS.z;


}
