#include "Control.h"

template<unsigned NUMBER_FINS>
Vector void FinControl<NUMBER_FINS>::get_desired_torque(const KinematicState& commanded_state, const KinematicState& estimated_state)
{
    Vector moment_arm_orientation_up = commanded_state.CS.z.cross(estimated_state.CS.z);

    double speed = estimated_state.velocity.norm();
    Vector correction_speed_orientation = estimated_state.CS.z);
}

template<>
void FinControl<3>::get_outputs(const KinematicState& commanded_state, const KinematicState& estimated_state, double time)
{

}

template<>
void FinControl<4>::get_outputs(const KinematicState& commanded_state, const KinematicState& estimated_state, double time)
{

}
