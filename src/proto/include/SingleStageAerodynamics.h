#pragma once

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

class SingleStageRocket;

struct SingleStageAerodynamics {

    double CD0;

    double CL_alpha;

    double CM_alpha;

    double CM_alpha_dot;

    double K;

    double ref_area;

    double ref_length;

    double stall_angle;

    double CL_max;

    double constant_term;

    double CM_max;

    Vector force;

    Vector moment;

    SingleStageRocket& rocket;

    SingleStageAerodynamics(SingleStageRocket& r);

    void set_coef(double* coef);

    void update();

};
