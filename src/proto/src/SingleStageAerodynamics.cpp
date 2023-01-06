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

    if(rocket.air.get_airspeed() < 1e-2){
        this->force.zero();
        this->moment.zero();
        return;
    }

    double CD0_compressible;

    if (rocket.air.get_mach() < 0.5) {
        CD0_compressible = this->CD0;
    } else {
        if(rocket.air.get_mach() > 1) {
            CD0_compressible = this->CD0 + this->CD0/rocket.air.get_mach();
        } else {
            CD0_compressible = this->CD0*(rocket.air.get_mach() - 0.5)*2.0;
        }
    }

    const auto& unit_v = rocket.air.get_air_velocity_unit_vector();

    double proj = rocket.CS.axis.z.dot(unit_v);

    this->moment = rocket.angular_velocity*(this->CM_alpha_dot*rocket.air.get_dynamic_pressure());

    if (proj > 0.99998) {
        this->force = unit_v*(CD0_compressible*rocket.air.get_dynamic_pressure());
        return;
    }

    double AoA = (proj > 0.9) ? sqrt(2 - 2*proj) : acos(proj);

    Vector arm = unit_v.cross(rocket.CS.axis.z);

    this->moment += arm*(this->CM_alpha*rocket.air.get_dynamic_pressure());

    double CL,CD;
    if(AoA > this->stall_angle) {
        CD = CD0_compressible + this->K*this->CL_max*this->CL_max*arm.norm()*this->constant_term;
        if(AoA > 2*this->stall_angle) {
            this->force = unit_v*(-CD*rocket.air.get_dynamic_pressure());
            return;
        } else {
            double frac = 1.0/(5*(AoA - this->stall_angle) + 1.0);
            CL = frac*this->CL_max;
        }
    } else {
        CL = this->CL_alpha*AoA;
        CD = CD0_compressible + this->K*CL*CL;
    }

    Vector lift = arm.cross(unit_v);
    double v = lift.norm();
    if(v > 1e-8) {
        CL /= v;
    }

    this->force = (lift*CL - unit_v*CD)*rocket.air.get_dynamic_pressure();
}
