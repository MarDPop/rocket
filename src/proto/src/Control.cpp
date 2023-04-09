#include "../include/Control.h"

#include "../include/Aerodynamics.h"

void Control::get_outputs(const Commands& commands, const KinematicState& estimated_state, double time) {}

template<unsigned NUMBER_FINS>
FinControlSimple<NUMBER_FINS>::FinControlSimple(const FinControlAero<NUMBER_FINS>& _aero) : aero(_aero) {}

template<unsigned NUMBER_FINS>
Vector FinControlSimple<NUMBER_FINS>::get_desired_arm_magnitude_body(const Commands& commands, const KinematicState& estimated_state)
{
    Vector angle_diff_inertial = commands.z_axis.cross(estimated_state.CS.axis.z);

    Vector arm_inertial = angle_diff_inertial * this->proportional - estimated_state.angular_velocity * this->damping;

    return estimated_state.CS * arm_inertial; // should have zero z component
}

template<unsigned NUMBER_FINS>
void FinControlSimple<NUMBER_FINS>::get_outputs(const Commands& commands, const KinematicState& estimated_state, double time)
{
    auto arm = this->get_desired_arm_magnitude(commands, estimated_state);

    for(unsigned fin_idx = 0; fin_idx < NUMBER_FINS; fin_idx++)
    {
        double angle = this->fin_gain*(arm.data[0]*aero.fin_span_vec_x[fin_idx] + arm.data[1]*aero.fin_span_vec_y[fin_idx]);
        this->aero.servos[fin_idx].set_commanded_angle(angle);
        this->aero.servos[fin_idx].update(time);
    }
}
