#include "../include/StageDynamics.h"

void StageDynamics::update_force_and_moment(const double& t) {
    this->gnc->update(t);
}
