#include "../include/Aerodynamics.h"

#include "../include/SingleStageRocket.h"

#include <cmath>

void Aerodynamics::compute_aero_values()
{
    Vector air_velocity = this->rocket.get_state().velocity - this->rocket.get_atmosphere().wind.wind;

    this->aero_values.airspeed = air_velocity.norm();

    if(this->aero_values.airspeed > 1e-3)
    {
        this->aero_values.unit_v_air = air_velocity * (1.0/this->aero_values.airspeed);
    }

    this->aero_values.mach = this->aero_values.airspeed * this->rocket.get_atmosphere().values.inv_sound_speed;

    double tmp = 1.0 + 0.2*this->aero_values.mach*this->aero_values.mach;
    this->aero_values.dynamic_pressure = this->rocket.get_atmosphere().values.pressure*(tmp*tmp*tmp*sqrt(tmp) - 1.0);
}

void Aerodynamics::compute_forces() {}

Aerodynamics::Aerodynamics(SingleStageRocket& r) : rocket(r) {}

Aerodynamics::~Aerodynamics(){}

void Aerodynamics::update()
{
    // Get aerodynamic quantities needed for computation
    this->compute_aero_values();

    this->force.zero();
    this->moment.zero();

    this->compute_forces();
}

AerodynamicsBasicCoef::AerodynamicsBasicCoef(SingleStageRocket& r) : Aerodynamics(r) {}

void AerodynamicsBasicCoef::set_coef(double* coef)
{
    this->CD0 = coef[0]*coef[5];
    this->CL_alpha = coef[1]*coef[5];
    this->CM_alpha = coef[2]*coef[5]*coef[6];
    this->CM_alpha_dot = coef[3]*coef[5]*coef[6];
    this->K = coef[4] / coef[5];
    this->ref_area = coef[5];
    this->ref_length = coef[6];
    this->stall_angle = coef[7];
    this->sin_stall_angle = sin(this->stall_angle);
    this->sin_zero_lift_angle = 2*this->sin_stall_angle;
    this->CL_max = this->CL_alpha*this->stall_angle;
    this->constant_term = 1.0/sin(this->stall_angle);
    this->CM_max = this->CM_alpha*2*this->stall_angle;

}

double AerodynamicsBasicCoef::get_parasitic_drag_from_mach(double mach)
{
    if (this->aero_values.mach < 0.5)
    {
        return this->CD0;
    }
    else
    {
        if(this->aero_values.mach > 1)
        {
            return this->CD0 + this->CD0/this->aero_values.mach;
        }
        else
        {
            return this->CD0*(this->aero_values.mach - 0.5)*2.0;
        }
    }
}

double AerodynamicsBasicCoef::get_angle_of_attack()
{
    double proj = std::min(1.0, rocket.get_state().CS.axis.z.dot(this->aero_values.unit_v_air));

    return (proj > 0.9) ? sqrt(2.0 - 2*proj) : acos(proj);
}

AerodynamicsBasicCoef::aero_coef AerodynamicsBasicCoef::get_aero_coef(double sAoA)
{
    AerodynamicsBasicCoef::aero_coef output;

    output.CD = this->get_parasitic_drag_from_mach(this->aero_values.mach);

    // for small angles of attack only compute parasitic drag (no lift or moments)
    if (sAoA < AOA_THRESHOLD)
    {
        return output;
    }

    output.CM = sAoA*this->CM_alpha;

    if(sAoA > this->sin_stall_angle)
    {
        output.CD += this->K*this->CL_max*this->CL_max*sAoA*this->constant_term; // norm of arm is equal to sin of angle which is approx lift curve
        if(sAoA > this->sin_zero_lift_angle)
        {
            return output;
        }
        else
        {
            double frac = 1.0/(5*(sAoA - this->sin_stall_angle) + 1.0);
            output.CL = frac*this->CL_max;
        }
    }
    else
    {
        output.CL = this->CL_alpha*sAoA;
        output.CD += this->K*output.CL*output.CL;
    }

    return output;
}

void AerodynamicsBasicCoef::compute_forces()
{
    // if airspeed too low don't compute, (has divide by zero consequences anyway)
    if(this->aero_values.airspeed < 1e-2)
    {
        return;
    }

    // already compute damping moment
    this->moment += rocket.get_state().angular_velocity*(this->CM_alpha_dot*this->rocket.get_atmosphere().values.density);

    // arm is the moment arm formed from the freestream, length of the arm is sin of angle between
    Vector arm = this->aero_values.unit_v_air.cross(rocket.get_state().CS.axis.z);

    // sin of the angle of attack
    double sAoA = arm.norm();

    auto coef = this->get_aero_coef(sAoA);

    Vector drag = this->aero_values.unit_v_air*(coef.CD*this->aero_values.dynamic_pressure);

    this->force -= drag;

    if(sAoA < AOA_THRESHOLD)
    {
        return;
    }

    // normalize arm
    arm *= (1.0/sAoA);

    Vector lift = arm.cross(this->aero_values.unit_v_air);

    this->moment -= arm*(coef.CM*this->aero_values.dynamic_pressure); // TODO: check orientations

    lift *= coef.CL/lift.norm();

    this->force += lift;
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

AerodynamicsFinCoefficient::~AerodynamicsFinCoefficient(){}

void AerodynamicsFinCoefficient::set_aero_coef(double dCL, double dCD, double dCM, double fin_COP_z, double fin_COP_d){
    this->dCLdTheta = dCL; // remember that these already have fin area "built in"
    this->dCDdTheta = dCD;
    this->dCMdTheta = dCM;
    this->z = fin_COP_z;
    this->d = fin_COP_d;
    this->const_axial_term_lift = dCL*fin_COP_d;
    this->const_axial_term_drag = dCD*fin_COP_d;
}

void AerodynamicsFinCoefficient::compute_forces()
{
    AerodynamicsBasicCoef::compute_forces();

    Vector dForce((char)0);
    Vector dMoment((char)0);

    double planar_term = this->dCMdTheta - (this->z - this->rocket.get_inertia().COG)*this->dCLdTheta;

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
    dMoment *= this->rocket.get_aerodynamics().aero_values.dynamic_pressure;
    dForce *= this->rocket.get_aerodynamics().aero_values.dynamic_pressure;
    // remember currently in body frame, need to convert to inertial frame
    this->moment += this->rocket.get_state().CS.transpose_mult(dMoment);
    this->force += this->rocket.get_state().CS.transpose_mult(dForce);
}
