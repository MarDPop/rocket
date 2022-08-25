#include "../include/Stage.h"

#include "../include/Vehicle.h"

Stage::Stage(const double& empty, const double& full, const std::array<double,6>& empty_i, const std::array<double,6>& full_i,const std::array<double,3>& empty_x, const std::array<double,3>& full_x) :
    mass_empty(empty), mass_full(full), inertia_empty(empty_i), inertia_full(full_i), COG_empty(empty_x), COG_full(full_x) {
    this->dm = full - empty;

    if(this->dm < 1e-3){
        this->is_ballistic = true;
        return;
    }

    this->is_ballistic = false;
    for(int i = 0; i < 3; i++) {
        this->dCGdm[i] = (full_x[i] - empty_x[i])/dm;
        this->dIdm[i] = (full_i[i] - empty_i[i])/dm;
    }

    for(int i = 3; i < 6; i++) {
        this->dIdm[i] = (full_i[i] - empty_i[i])/dm;
    }

    if(fabs(full_i[3]) < 1e-6 && fabs(full_i[5]) < 1e-6) {
        this->is_plane = true;
    } else {
        this->is_plane = false;
    }

    if(this->is_plane && fabs(full_i[4]) < 1e-6) {
        this->is_symmetric = true;
    } else {
        this->is_symmetric = false;
    }

    this->actions.push_back(this->GNC); // GNC must be first
    this->actions.push_back(this->aero);
    this->actions.push_back(this->thruster);
}

void Stage::set_mass(double mass) {

    double dm = mass - this->mass_empty;

    unsigned int i = 0;
    while(i < 3) {
        this->vehicle->COG[i] = this->COG_empty[i] + this->dCGdm[i]*dm;
        this->vehicle->inertia[i] = this->inertia_empty[i] + this->dIdm[i]*dm;
        i++;
    }

    if(this->is_symmetric) {
        return;
    }

    while(i < 6) {
        this->vehicle->inertia[i] = this->inertia_empty[i] + this->dIdm[i]*dm;
        i++;
    }

}

void Stage::compute() {
    this->vehicle->force.zero();
    this->vehicle->moment.zero();

    for(auto& a : this->actions) {
        a.update(this->vehicle->Talo);
        Vector torque = a.center.cross(a.force);
        for(int i = 0; i < 3; i++) {
            this->vehicle->force.data[i] += a.force.data[i];
            this->vehicle->moment.data[i] += a.moment.data[i] + torque.data[i];
        }
    }
}
