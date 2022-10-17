#include "../include/Aerodynamics.h"
#include "../include/Vehicle.h"
#include "../../common/include/Cartesian.h"

using namespace Cartesian;

void Aerodynamics::set_vehicle(Vehicle* vehicle) {
    this->vehicle = vehicle;
    this->air = &(vehicle->body.atmosphere->air);
}

AerodynamicsDragOnly::AerodynamicsDragOnly(double CD, double A) : CD_A(CD*A*-0.5) {}

AerodynamicsDragOnly::~AerodynamicsDragOnly() {}

void AerodynamicsDragOnly::update(double time) {

    Vector air_vel_in_body = this->vehicle->body.body_fixed_CS * this->vehicle->body.body_fixed_velocity;

    double drag_v = this->CD_A*this->air->density*air_vel_in_body.norm();

    this->force = air_vel_in_body * drag_v;
}

void AerodynamicsBasic::update(double time) {

    //Vector aoa = this->vehicle->planet.ECEF.axis

}


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
    Vector airspeed_vec = rocket.velocity;
    if(rocket.wind){
        airspeed_vec -= rocket.wind->wind;
    }

    double v2 = airspeed_vec.dot(airspeed_vec);

    if(v2 < 1e-2){
        this->force.zero();
        this->moment.zero();
        return;
    }

    double airspeed = sqrt(v2);

    double mach = airspeed * rocket.sound_speed_inv;

    double CD0_compressible;

    if (mach < 0.5) {
        CD0_compressible = this->CD0;
    } else {
        if(mach > 1) {
            CD0_compressible = this->CD0 + this->CD0/mach;
        } else {
            CD0_compressible = this->CD0*(mach - 0.5)*2.0;
        }
    }

    Vector unit_v = airspeed_vec * (1.0/airspeed);

    double proj = rocket.CS.axis.z.dot(unit_v);

    v2 *= rocket.air_density*0.5;

    this->moment = rocket.angular_velocity*(this->CM_alpha_dot*rocket.air_density);

    if (proj > 0.99998) {
        this->force = unit_v*(CD0_compressible*-v2);
        return;
    }

    double AoA = (proj > 0.9) ? sqrt(2 - 2*proj) : acos(proj);

    Vector arm = unit_v.cross(rocket.CS.axis.z);

    this->moment += arm*(this->CM_alpha*v2);

    double CL,CD;
    if(AoA > this->stall_angle) {
        CD = CD0_compressible + this->K*this->CL_max*this->CL_max*arm.norm()*this->constant_term;
        if(AoA > 2*this->stall_angle) {
            this->force = unit_v*(-CD*v2);
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

    this->force = (lift*CL - unit_v*CD)*v2;
}
