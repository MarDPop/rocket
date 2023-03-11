#include "../include/Aerodynamics.h"

#include "../include/SingleStageRocket.h"

Aerodynamics::Aerodynamics(SingleStageRocket& r) : rocket(r) {}

void Aerodynamics::compute_aero_values()
{
    Vector air_velocity = this->rocket.state.velocity - this->rocket.altitude_table.wind.wind;

    this->aero_values.airspeed = air_velocity.norm();

    if(this->aero_values.airspeed > 1e-3)
    {
        this->aero_values.unit_v_air = air_velocity * (1.0/this->aero_values.airspeed);
    }

    this->aero_values.mach = this->aero_values.airspeed * this->rocket.altitude_table.values->inv_sound_speed;

    double tmp = 1.0 + 0.2*this->aero_values.mach*this->aero_values.mach;
    this->aero_values.dynamic_pressure = this->rocket.altitude_table.values->pressure*(tmp*tmp*tmp*sqrt(tmp) - 1.0);
}

SimpleAerodynamics::SimpleAerodynamics(SingleStageRocket& r) : Aerodynamics(r) {}

void SimpleAerodynamics::set_coef(double* coef)
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

double SimpleAerodynamics::get_parasitic_drag_from_mach(double mach)
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

double SimpleAerodynamics::get_angle_of_attack()
{
    double proj = std::min(1.0, rocket.state.CS.axis.z.dot(this->aero_values.unit_v_air));

    return (proj > 0.9) ? sqrt(2.0 - 2*proj) : acos(proj);
}

SimpleAerodynamics::aero_coef SimpleAerodynamics::get_aero_coef(double sAoA)
{
    SimpleAerodynamics::aero_coef output;

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

void SimpleAerodynamics::update()
{
    // Get aerodynamic quantities needed for computation
    this->compute_aero_values();

    this->force.zero();
    this->moment.zero();

    // if airspeed too low don't compute, (has divide by zero consequences anyway)
    if(this->aero_values.airspeed < 1e-2)
    {
        return;
    }

    // already compute damping moment
    this->moment += rocket.state.angular_velocity*(this->CM_alpha_dot*this->rocket.altitude_table.values->density);

    // arm is the moment arm formed from the freestream, length of the arm is sin of angle between
    Vector arm = this->aero_values.unit_v_air.cross(rocket.state.CS.axis.z);

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

void ControlledAerodynamics::deflect_fins(double time)
{
    double max_angle = this->slew_rate*(time - this->time_old);
    for(unsigned i = 0; i < NFINS;i++) {
        auto& fin = this->fins[i];
        double delta = fin.commanded_deflection - fin.deflection;
        if(fabs(delta) > max_angle) {
            fin.deflection += std::copysign(max_angle,delta);
        } else {
            fin.deflection = fin.commanded_deflection;
        }

        fin.deflection = std::copysign(std::min(fabs(fin.deflection),this->max_theta),fin.deflection);
    }
}

void FinCoefficientAerodynamics::set_aero_coef(double dCL, double dCD, double dCM, double fin_COP_z, double fin_COP_d){
    this->dCLdTheta = dCL; // remember that these already have fin area "built in"
    this->dCDdTheta = dCD;
    this->dCMdTheta = dCM;
    this->z = fin_COP_z;
    this->d = fin_COP_d;
    this->const_axial_term = dCL*fin_COP_d;
    this->const_planer_term = dCM - (z - this->rocket->inertia.COG)*dCL; // remember z should be negative distance from nose
}




void FinCoefficientAerodynamics::update_force() {
    this->dMoment.zero();
    this->dForce.zero();

    double axial_term = this->dCLdTheta*this->d;
    double planar_term = this->dCMdTheta - (this->z - this->rocket->inertia.COG)*this->dCLdTheta;

    for(unsigned i = 0; i < NFINS;i++) {
        auto& fin = this->fins[i];
        double tmp = planar_term*fin.deflection;

        this->dMoment.data[0] += fin.span.data[0]*tmp;
        this->dMoment.data[1] += fin.span.data[1]*tmp;
        this->dMoment.data[2] += fin.span.data[2]*axial_term*fin.deflection;

        tmp = this->dCLdTheta*fin.deflection;

        this->dForce.data[0] += fin.lift.data[0]*tmp;
        this->dForce.data[1] += fin.lift.data[1]*tmp;
        this->dForce.data[2] -= this->dCDdTheta*fin.deflection; // simply linear approximation for small angles
    }
    this->dMoment *= this->rocket->aerodynamics->aero_values.dynamic_pressure;
    this->dForce *= this->rocket->aerodynamics->aero_values.dynamic_pressure;
    // remember currently in body frame, need to convert to inertial frame
    this->dMoment = this->rocket->state.CS.transpose_mult(this->dMoment);
    this->dForce = this->rocket->state.CS.transpose_mult(this->dForce);
}
