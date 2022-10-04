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

    this->CD0 = coef[0];
    this->CL_alpha = coef[1];
    this->CM_alpha = coef[2];
    this->K = coef[3];
    this->ref_area = coef[4];
    this->stall_angle = coef[5];

}

void SingleStageAerodynamics::update() {
    double v2 = rocket.velocity.dot(rocket.velocity);
    double airspeed = sqrt(v2);

    Vector unit_v = rocket.velocity * (1.0/airspeed);

    Vector arm = rocket.CS.axis.z.cross(unit_v);

    double AoA = sqrt(1 - rocket.CS.axis.z.dot(unit_v));

    double mach = airspeed * rocket.sound_speed_inv;

    double CD0_compressible;

    if (mach < 0.5) {
        CD0_compressible = this->CD0;
    }

    double CM;
    double CL = this->CL_alpha*AoA;
    double CD = CD0_compressible + this->K*CL*CL;
    if(AoA < this->stall_angle) {
        CM = AoA*this->CM_alpha;
    } else {
        CM = 0;
        CL = 0;
    }

    v2 *= this->ref_area*rocket.density*0.5;

    this->moment = arm*(CM*v2);
    this->force =
}
