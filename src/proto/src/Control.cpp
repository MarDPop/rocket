#include "Control.h"

void Control::get_outputs(const Commands& commands, const KinematicState& estimated_state, double time) {}

template<unsigned NUMBER_FINS>
Vector void FinControl<NUMBER_FINS>::get_desired_torque(const Commands& commands, const KinematicState& estimated_state)
{
    Vector delta_angular_velocity = commands.angular_velocity_in_body - estimated_state.angular_velocity;


}
