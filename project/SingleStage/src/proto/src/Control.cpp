#include "../include/Control.h"

#include "../include/Aerodynamics.h"

Control::Control() {}

Control::~Control() {}

void Control::get_outputs(const Commands& commands, const KinematicState& estimated_state, const Sensors* sensors, double time) {}

ControlFinSimple::ControlFinSimple(FinControlAero& _aero) : aero(_aero)
{

}

ControlFinSimple::~ControlFinSimple(){}

Vector ControlFinSimple::get_desired_arm_magnitude_body(const Commands& commands, const KinematicState& estimated_state)
{
    Vector angle_diff_inertial = commands.z_axis_inertial.cross(estimated_state.CS.axis.z);

    Vector arm_body = estimated_state.CS * angle_diff_inertial;

    return (arm_body * this->proportional) - (estimated_state.angular_velocity * this->damping); // should have zero z component
}

void ControlFinSimple::get_outputs(const Commands& commands, const KinematicState& estimated_state, const Sensors* sensors, double time)
{
    double dynamic_pressure_factor = sensors->get_measured_dynamic_pressure();

    if(dynamic_pressure_factor < 0.1)
    {
        return;
    }

    auto arm = this->get_desired_arm_magnitude_body(commands, estimated_state);

    double gain = this->fin_gain/dynamic_pressure_factor;

    for(unsigned fin_idx = 0; fin_idx < aero.NUMBER_FINS; fin_idx++)
    {
        const Fin& fin = aero.get_fin(fin_idx);
        double angle = gain*(arm.x*fin.span_x + arm.y*fin.span_y);
        fin.servo->set_commanded_angle(angle);
        fin.servo->update(time);
    }
}
