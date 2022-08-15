#include "../include/Stage.h"

Stage::Stage(const double& empty, const double& full, const std::array<double,6>& empty_i, const std::array<double,6>& full_i,const std::array<double,3>& empty_x, const std::array<double,3>& full_x) :
    mass_empty(empty), mass_full(full), inertia_empty(empty_i), inertia_full(full_i), COG_empty(empty_x), COG_full(full_x) {
    this->dm = full - empty;

    if(this->dm < 1e-3){
        this->is_ballistic = true;
        return;
    }

    this->is_ballistic = false;
    for(int i = 0; i < 3; i++) {
        this->dCGdm[i] = (full_COG[i] - empty_COG[i])/dm;
        this->dIdm[i] = (full_inertia[i] - empty_inertia[i])/dm;
    }

    for(int i = 3; i < 6; i++) {
        this->dIdm[i] = (full_inertia[i] - empty_inertia[i])/dm;
    }

    if(fabs(this->full_inertia[3]) < 1e-6 && fabs(this->full_inertia[5]) < 1e-6) {
        this->is_plane = true;
    } else {
        this->is_plane = false;
    }

    if(this->is_plane && fabs(this->full_inertia[4]) < 1e-6) {
        this->is_symmetric = true;
    } else {
        this->is_symmetric = false;
    }
}

void Stage::set_mass(double mass) {
    this->mass = mass;

    double dm = mass - this->mass_empty;

    int i = 0;
    while(i < 3) {
        this->center[i] = this->COG_empty[i] + this->dCGdm[i]*dm;
        this->inertia[i] = this->inertia_empty[i] + this->dIdm[i]*dm;
        i++;
    }

    if(this->is_symmetric) {
        return;
    }

    while(i < 6) {
        this->inertia[i] = this->inertia_empty[i] + this->dIdm[i]*dm;
        i++;
    }

}

void Stage::update_force_and_moment() {
    for(Action& a : this->actions) {

    }
}
