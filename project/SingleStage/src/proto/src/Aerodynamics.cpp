#include "../include/Aerodynamics.h"

#include "../include/SingleStageRocket.h"

#include <cmath>
#include <string>

void Aerodynamics::compute_aero_values()
{
    Vector air_velocity = this->rocket.get_state().velocity - this->rocket.get_atmosphere().wind.wind;

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

    this->_aero_values.mach = this->_aero_values.airspeed * this->rocket.get_atmosphere().values.inv_sound_speed;

    double tmp = 1.0 + 0.2*this->_aero_values.mach*this->_aero_values.mach;
    this->_aero_values.dynamic_pressure = this->rocket.get_atmosphere().values.pressure*(tmp*tmp*tmp*sqrt(tmp) - 1.0);
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
    this->CD0 = coef[0]*coef[5];
    this->CL_alpha = coef[1]*coef[5];
    this->CM_alpha = coef[2]*coef[5]*coef[6];
    this->CM_alpha_dot = coef[3]*coef[5]*coef[6];
    this->K = coef[4] / coef[5];
    this->ref_area = coef[5];
    this->ref_length = coef[6];
    this->stall_angle = coef[7];
    this->_action.location.data[2] = coef[8];
    this->CL_max = this->CL_alpha*this->stall_angle;
    this->CM_max = this->CM_alpha*this->stall_angle;
}

double AerodynamicsBasicCoefficient::get_parasitic_drag_from_mach(double mach)
{
    if (this->_aero_values.mach < 0.5)
    {
        return this->CD0;
    }
    else
    {
        if(this->_aero_values.mach > 1)
        {
            return this->CD0 + this->CD0/this->_aero_values.mach;
        }
        else
        {
            return this->CD0*(this->_aero_values.mach - 0.5)*2.0;
        }
    }
}

double AerodynamicsBasicCoefficient::get_angle_of_attack()
{
    const double& proj = this->_aero_values.unit_v_air_body.data[2];

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

    output.CM = std::min(this->CM_max,this->CM_alpha*sAoA);
    output.CL = this->CL_alpha*sAoA;
    output.CD += this->K*output.CL*output.CL;
    if(output.CL > this->CL_max)
    {
        output.CL = this->CL_max;
    }
    return output;
}

void AerodynamicsBasicCoefficient::compute_forces()
{
    // if airspeed too low don't compute, (has divide by zero consequences anyway)
    if(this->_aero_values.airspeed < 1e-2)
    {
        return;
    }

    // already compute damping moment
    Vector angular_velocity_body = rocket.get_state().CS * rocket.get_state().angular_velocity;
    this->_action.moment += angular_velocity_body*(this->CM_alpha_dot*this->rocket.get_atmosphere().values.density);

    // arm is the moment arm formed from the freestream, length of the arm is sin of angle between
    Vector arm(this->_aero_values.unit_v_air_body.data[1], -this->_aero_values.unit_v_air_body.data[0],0.0);

    // sin of the angle of attack
    double sAoA = sqrt(arm.data[0]*arm.data[0] + arm.data[1]*arm.data[1]);

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
        fin.servo = std::make_unique<Servo>();
        angle += dAngle;
    }
}

AerodynamicsFinCoefficient::AerodynamicsFinCoefficient(SingleStageRocket& r, unsigned NFINS) : Aerodynamics(r), AerodynamicsBasicCoefficient(r), FinControlAero(NFINS) {}

AerodynamicsFinCoefficient::~AerodynamicsFinCoefficient(){}

void AerodynamicsFinCoefficient::set_fin_coef(const std::array<double,6>& coef){
    this->dCLdTheta = coef[0]*coef[3];
    this->dCDdTheta = coef[1]*coef[3];
    this->dCMdTheta = coef[2]*coef[3];
    this->z = coef[4];
    this->d = coef[5];
    this->const_axial_term_lift = this->dCLdTheta*this->d;
    this->const_axial_term_drag = this->dCDdTheta*this->d;
}

void AerodynamicsFinCoefficient::compute_forces()
{
    AerodynamicsBasicCoefficient::compute_forces();

    Vector dForce((char)0);
    Vector dMoment((char)0);

    double planar_term = this->dCMdTheta - (this->z - this->rocket.get_inertia().CoM.z)*this->dCLdTheta;

    for(Fin& fin : this->fins)
    {
        double deflection = fin.servo->get_angle();

        dMoment.data[0] += (fin.span_x*planar_term - fin.span_y*this->const_axial_term_drag)*deflection;
        dMoment.data[1] += (fin.span_y*planar_term + fin.span_x*this->const_axial_term_drag)*deflection;
        dMoment.data[2] += this->const_axial_term_lift*deflection;

        double tmp = this->dCLdTheta*deflection;

        dForce.data[0] -= fin.span_y*tmp;
        dForce.data[1] += fin.span_x*tmp;
        dForce.data[2] -= this->dCDdTheta*deflection; // simply linear approximation for small angles
    }
    dMoment *= this->_aero_values.dynamic_pressure;
    dForce *= this->_aero_values.dynamic_pressure;
    // remember currently in body frame, need to convert to inertial frame
    this->_action.moment += this->rocket.get_state().CS.transpose_mult(dMoment);
    this->_action.force += this->rocket.get_state().CS.transpose_mult(dForce);
}
