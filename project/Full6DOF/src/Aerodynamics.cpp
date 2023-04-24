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

    Eigen::Vector3d air_vel_in_body = this->vehicle->body.body_fixed_CS * this->vehicle->body.body_fixed_velocity;

    double drag_v = this->CD_A*this->air->density*air_vel_in_body.norm();

    this->force = air_vel_in_body * drag_v;
}

void AerodynamicsBasic::update(double time) {

    //Vector aoa = this->vehicle->planet.ECEF.axis

}

template <unsigned N_CONTROL>
AerodynamicsTable<N_CONTROL>::AerodynamicsTable(std::string fn) : table(NestedTable<4+N_CONTROL,6>::create(fn)) {}
