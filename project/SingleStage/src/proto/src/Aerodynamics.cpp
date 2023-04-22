#include "../include/Aerodynamics.h"

#include "../include/SingleStageRocket.h"

#include <cmath>
#include <string>

void Aerodynamics::compute_aero_values()
{
    Vector air_velocity = this->rocket.get_state().velocity - this->rocket.get_environment().wind.wind;

    this->_aero_values.airspeed = air_velocity.norm();

    if(this->_aero_values.airspeed > 1e-3)
    {
        this->_aero_values.unit_v_air_body = (this->rocket.get_state().CS*air_velocity) * (1.0/this->_aero_values.airspeed);
    }
    else
    {
        this->_aero_values.unit_v_air_body.data[0] = 0.0;
        this->_aero_values.unit_v_air_body.data[1] = 0.0;
        this->_aero_values.unit_v_air_body.data[2] = -1.0;
    }

    this->_aero_values.mach = this->_aero_values.airspeed * this->rocket.get_environment().values.inv_sound_speed;

    double tmp = 1.0 + 0.2*this->_aero_values.mach*this->_aero_values.mach;
    this->_aero_values.dynamic_pressure = this->rocket.get_environment().values.pressure*(tmp*tmp*tmp*sqrt(tmp) - 1.0);
}

void Aerodynamics::compute_forces() {}

Aerodynamics::Aerodynamics(SingleStageRocket& r) : rocket(r)
{
    this->_action.location.zero();
}

Aerodynamics::~Aerodynamics(){}

const BodyAction& Aerodynamics::update()
{
    // Get aerodynamic quantities needed for computation
    this->compute_aero_values();

    this->_action.force.zero();
    this->_action.moment.zero();

    this->compute_forces();

    return this->_action; // location 0
}

AerodynamicsBasicCoefficient::AerodynamicsBasicCoefficient(SingleStageRocket& r) : Aerodynamics(r) {}

AerodynamicsBasicCoefficient::~AerodynamicsBasicCoefficient() {}

void AerodynamicsBasicCoefficient::set_coef(const std::array<double,9>& coef)
{
    this->_CD0 = coef[0]*coef[5];
    this->_CL_alpha = coef[1]*coef[5];
    this->_CM_alpha = coef[2]*coef[5]*coef[6];
    this->_CM_alpha_dot = coef[3]*coef[5]*coef[6];
    this->_K = coef[4] / coef[5];
    this->_ref_area = coef[5];
    this->_ref_length = coef[6];
    this->_stall_angle = coef[7];
    this->_action.location.z = coef[8];
    this->_CL_max = this->_CL_alpha*this->_stall_angle;
    this->_CM_max = this->_CM_alpha*this->_stall_angle;
}

double AerodynamicsBasicCoefficient::get_parasitic_drag_from_mach(double mach)
{
    if (this->_aero_values.mach < 0.5)
    {
        return this->_CD0;
    }
    else
    {
        if(this->_aero_values.mach > 1)
        {
            return this->_CD0 + this->_CD0/this->_aero_values.mach;
        }
        else
        {
            return this->_CD0*(this->_aero_values.mach - 0.5)*2.0;
        }
    }
}

double AerodynamicsBasicCoefficient::get_angle_of_attack()
{
    const double& proj = this->_aero_values.unit_v_air_body.z;

    return (proj > 0.9) ? sqrt(2.0*std::max(1.0 - proj,0.0)) : acos(proj);
}

AerodynamicsBasicCoefficient::aero_coef AerodynamicsBasicCoefficient::get_aero_coef(double sAoA)
{
    AerodynamicsBasicCoefficient::aero_coef output;

    output.CD = this->get_parasitic_drag_from_mach(this->_aero_values.mach);

    // for small angles of attack only compute parasitic drag (no lift or moments)
    if (sAoA < AOA_THRESHOLD)
    {
        return output;
    }

    output.CM = std::min(this->_CM_max,this->_CM_alpha*sAoA);
    output.CL = this->_CL_alpha*sAoA;
    output.CD += this->_K*output.CL*output.CL;
    if(output.CL > this->_CL_max)
    {
        output.CL = this->_CL_max;
    }
    return output;
}

void AerodynamicsBasicCoefficient::compute_forces()
{
    // if airspeed too low don't compute, (has divide by zero consequences anyway)
    if(this->_aero_values.airspeed < AIRSPEED_THRESHOLD)
    {
        return;
    }

    double angular_rate = rocket.get_state().angular_velocity.norm();
    this->_action.moment += rocket.get_state().angular_velocity*(this->_CM_alpha_dot*this->rocket.get_environment().values.density*angular_rate);

    // arm is the moment arm formed from the freestream, length of the arm is sin of angle between
    Vector arm(this->_aero_values.unit_v_air_body.y, -this->_aero_values.unit_v_air_body.x,0.0);

    // sin of the angle of attack
    double sAoA = sqrt(arm.x*arm.x + arm.y*arm.y);

    auto coef = this->get_aero_coef(sAoA);

    Vector drag = this->_aero_values.unit_v_air_body*(coef.CD*this->_aero_values.dynamic_pressure);

    this->_action.force -= drag;

    if(sAoA < AOA_THRESHOLD)
    {
        return;
    }

    // normalize arm
    arm *= (1.0/sAoA);

    Vector lift = arm.cross(this->_aero_values.unit_v_air_body);

    this->_action.moment -= arm*(coef.CM*this->_aero_values.dynamic_pressure); // TODO: check orientations

    lift *= coef.CL*this->_aero_values.dynamic_pressure/lift.norm();

    this->_action.force += lift;

}

FinControlAero::FinControlAero(unsigned NFINS) : fins(NFINS), NUMBER_FINS(NFINS)
{
    double angle = 0;
    double dAngle = 6.283185307179586476925286766559/NFINS;
    for(Fin& fin : this->fins)
    {
        fin.span_x = cos(angle);
        fin.span_y = sin(angle);
        fin.servo = std::make_unique<BoundedServo>(-0.1,0.1);
        angle += dAngle;
    }
}

AerodynamicsFinCoefficient::AerodynamicsFinCoefficient(SingleStageRocket& r, unsigned NFINS) : Aerodynamics(r), AerodynamicsBasicCoefficient(r), FinControlAero(NFINS) {}

AerodynamicsFinCoefficient::~AerodynamicsFinCoefficient(){}

void AerodynamicsFinCoefficient::set_fin_coef(const std::array<double,6>& coef){
    this->dCLdTheta = coef[0]*coef[3];
    this->dCDdTheta = coef[1]*coef[3];
    this->dCMdTheta = coef[2]*coef[3];
    this->delta_z = coef[4] - this->_action.location.z;
    this->span_d = coef[5];
    this->const_axial_term_lift = this->dCLdTheta*this->span_d;
    this->const_axial_term_drag = this->dCDdTheta*this->span_d;
    this->const_axial_term_moment = this->dCMdTheta - delta_z*this->dCLdTheta; // TODO: check sign
}

void AerodynamicsFinCoefficient::compute_forces()
{
    AerodynamicsBasicCoefficient::compute_forces();

    Vector dForce((char)0);
    Vector dMoment((char)0);

    for(Fin& fin : this->fins)
    {
        double deflection = fin.servo->get_angle();

        dMoment.data[0] += (fin.span_x*this->const_axial_term_moment - fin.span_y*this->const_axial_term_drag)*deflection;
        dMoment.data[1] += (fin.span_y*this->const_axial_term_moment + fin.span_x*this->const_axial_term_drag)*deflection;
        dMoment.data[2] += this->const_axial_term_lift*deflection;

        double tmp = this->dCLdTheta*deflection;

        dForce.data[0] -= fin.span_y*tmp;
        dForce.data[1] += fin.span_x*tmp;
        dForce.data[2] -= this->dCDdTheta*deflection; // simply linear approximation for small angles
    }
    dMoment *= this->_aero_values.dynamic_pressure;
    dForce *= this->_aero_values.dynamic_pressure;
    this->_action.moment += dMoment;
    this->_action.force += dForce;
}
