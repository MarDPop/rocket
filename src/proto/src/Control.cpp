#include "Control.h"

void Control::get_outputs(const Commands& commands, const KinematicState& estimated_state, double time) {}

template<unsigned NUMBER_FINS>
Vector FinControl<NUMBER_FINS>::get_desired_arm_magnitude(const Commands& commands, const KinematicState& estimated_state)
{
    Vector angle_diff = commands.axis_orientation.cross(estimated_state.CS.z);

    return angle_diff * this->proportional - estimated_state.angular_velocity * this->damping;
}
