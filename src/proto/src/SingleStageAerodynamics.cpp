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

void SingleStageAerodynamics::update() {

    if(rocket.airspeed < 1e-2){
        this->force.zero();
        this->moment.zero();
        return;
    }

    double CD0_compressible;

    if (rocket.mach < 0.5) {
        CD0_compressible = this->CD0;
    } else {
        if(rocket.mach > 1) {
            CD0_compressible = this->CD0 + this->CD0/rocket.mach;
        } else {
            CD0_compressible = this->CD0*(rocket.mach - 0.5)*2.0;
        }
    }

    double proj = rocket.CS.axis.z.dot(rocket.air.unit_v_air);

    this->moment = rocket.angular_velocity*(this->CM_alpha_dot*rocket.air.dynamic_pressure);

    if (proj > 0.99998) {
        this->force = rocket.air.unit_v_air*(CD0_compressible*rocket.air.dynamic_pressure);
        return;
    }

    double AoA = (proj > 0.9) ? sqrt(2 - 2*proj) : acos(proj);

    Vector arm = rocket.air.unit_v_air.cross(rocket.CS.axis.z);

    this->moment += arm*(this->CM_alpha*v2);

    double CL,CD;
    if(AoA > this->stall_angle) {
        CD = CD0_compressible + this->K*this->CL_max*this->CL_max*arm.norm()*this->constant_term;
        if(AoA > 2*this->stall_angle) {
            this->force = rocket.air.unit_v_air*(-CD*rocket.air.dynamic_pressure);
            return;
        } else {
            double frac = 1.0/(5*(AoA - this->stall_angle) + 1.0);
            CL = frac*this->CL_max;
        }
    } else {
        CL = this->CL_alpha*AoA;
        CD = CD0_compressible + this->K*CL*CL;
    }

    Vector lift = arm.cross(rocket.air.unit_v_air);
    double v = lift.norm();
    if(v > 1e-8) {
        CL /= v;
    }

    this->force = (lift*CL - rocket.air.unit_v_air*CD)*rocket.air.dynamic_pressure;
}
