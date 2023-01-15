#include "../include/SingleStageAerodynamics.h"

#include "../include/SingleStageRocket.h"

SingleStageAerodynamics::SingleStageAerodynamics(SingleStageRocket& r) : rocket(r) {

}

void SingleStageAerodynamics::set_coef(double* coef) {

    this->CD0 = coef[0]*coef[5];
    this->CL_alpha = coef[1]*coef[5];
    this->CM_alpha = coef[2]*coef[5]*coef[6];
    this->CM_alpha_dot = coef[3]*coef[5]*coef[6];
    this->K = coef[4] / coef[5];
    this->ref_area = coef[5];
    this->ref_length = coef[6];
    this->stall_angle = coef[7];
    this->CL_max = this->CL_alpha*this->stall_angle;
    this->constant_term = 1.0/sin(this->stall_angle);
    this->CM_max = this->CM_alpha*2*this->stall_angle;

}

void SingleStageAerodynamics::compute_aero_values() {
    Vector air_velocity = this->rocket.state.velocity - this->rocket.altitude_table.wind.wind;

    this->aero_values.airspeed = air_velocity.norm();

    this->aero_values.unit_v_air = air_velocity * (1.0/this->aero_values.airspeed);

    this->aero_values.mach = this->aero_values.airspeed * this->rocket.altitude_table.values->inv_sound_speed;

    double tmp = 1.0 + 0.2*this->aero_values.mach*this->aero_values.mach;
    this->aero_values.dynamic_pressure = this->rocket.altitude_table.values->pressure*(tmp*tmp*tmp*sqrt(tmp) - 1.0);
}

void SingleStageAerodynamics::update() {

    this->compute_aero_values();

    if(this->aero_values.airspeed < 1e-2){
        this->force.zero();
        this->moment.zero();
        return;
    }

    double CD0_compressible;

    if (this->aero_values.mach < 0.5) {
        CD0_compressible = this->CD0;
    } else {
        if(this->aero_values.mach > 1) {
            CD0_compressible = this->CD0 + this->CD0/this->aero_values.mach;
        } else {
            CD0_compressible = this->CD0*(this->aero_values.mach - 0.5)*2.0;
        }
    }

    double proj = rocket.state.CS.axis.z.dot(this->aero_values.unit_v_air);

    this->moment = rocket.state.angular_velocity*(this->CM_alpha_dot*this->aero_values.dynamic_pressure);

    if (proj > 0.99998) {
        this->force = this->aero_values.unit_v_air*(CD0_compressible*this->aero_values.dynamic_pressure);
        return;
    }

    double AoA = (proj > 0.9) ? sqrt(2 - 2*proj) : acos(proj);

    Vector arm = this->aero_values.unit_v_air.cross(rocket.state.CS.axis.z);

    this->moment += arm*(this->CM_alpha*this->aero_values.dynamic_pressure);

    double CL,CD;
    if(AoA > this->stall_angle) {
        CD = CD0_compressible + this->K*this->CL_max*this->CL_max*arm.norm()*this->constant_term;
        if(AoA > 2*this->stall_angle) {
            this->force = this->aero_values.unit_v_air*(-CD*this->aero_values.dynamic_pressure);
            return;
        } else {
            double frac = 1.0/(5*(AoA - this->stall_angle) + 1.0);
            CL = frac*this->CL_max;
        }
    } else {
        CL = this->CL_alpha*AoA;
        CD = CD0_compressible + this->K*CL*CL;
    }

    Vector lift = arm.cross(this->aero_values.unit_v_air);
    double v = lift.norm();
    if(v > 1e-8) {
        CL /= v;
    }

    this->force = (lift*CL - this->aero_values.unit_v_air*CD)*this->aero_values.dynamic_pressure;
}
