#include "../include/Control.h"

#include "../include/Aerodynamics.h"

Control::Control() {}

Control::~Control() {}

void Control::get_outputs(const Commands& commands, const KinematicState& estimated_state, double time) {}

ControlFinSimple::ControlFinSimple(FinControlAero& _aero) : aero(_aero)
{

}

ControlFinSimple::~ControlFinSimple(){}

Vector ControlFinSimple::get_desired_arm_magnitude_body(const Commands& commands, const KinematicState& estimated_state)
{
    Vector angle_diff_inertial = commands.z_axis_inertial.cross(estimated_state.CS.axis.z);

    Vector arm_inertial = angle_diff_inertial * this->proportional - estimated_state.angular_velocity * this->damping;

    return estimated_state.CS * arm_inertial; // should have zero z component
}

void ControlFinSimple::get_outputs(const Commands& commands, const KinematicState& estimated_state, double time)
{
    auto arm = this->get_desired_arm_magnitude_body(commands, estimated_state);

    for(unsigned fin_idx = 0; fin_idx < aero.NUMBER_FINS; fin_idx++)
    {
        const Fin& fin = aero.get_fin(fin_idx);
        double angle = this->fin_gain*(arm.x*fin.span_x + arm.y*fin.span_y);
        fin.servo->set_commanded_angle(angle);
        fin.servo->update(time);
    }
}
