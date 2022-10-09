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

    this->CD0 = coef[0]*coef[4];
    this->CL_alpha = coef[1]*coef[4];
    this->CM_alpha = coef[2]*coef[4]*coef[5];
    this->K = coef[3] / coef[4];
    this->ref_area = coef[4];
    this->ref_length = coef[5];
    this->stall_angle = coef[6];

}

void SingleStageAerodynamics::update() {
    double v2 = rocket.velocity.dot(rocket.velocity);

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

    Vector unit_v = rocket.velocity * (1.0/airspeed);

    double proj = rocket.CS.axis.z.dot(unit_v);

    if (proj > 0.9999) {
        this->moment.zero();
        this->force = unit_v*(CD0_compressible*-0.5*rocket.air_density*v2);
        return;
    }

    double AoA = acos(rocket.CS.axis.z.dot(unit_v));

    Vector arm = rocket.CS.axis.z.cross(unit_v);

    double CM;
    double CL = this->CL_alpha*AoA;
    double CD = CD0_compressible + this->K*CL*CL;
    if(AoA < this->stall_angle) {
        CM = AoA*this->CM_alpha;
    } else {
        CM = 0;
        CL = 0;
    }

    v2 *= rocket.air_density*0.5;

    Vector lift = unit_v.cross(arm);
    double v = lift.norm();
    if(v > 1e-6) {
        lift *= (1.0/v);
    }

    this->moment = arm*(CM*v2); // arm length build into CM
    this->force = (lift*CL - unit_v*CD)*v2;
}
