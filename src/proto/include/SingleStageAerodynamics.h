#pragma once

#include "Atmosphere.h"

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

class SingleStageRocket;

struct AeroValues {
    /**
    * current air speed (m/s)
    */
    double airspeed;

    /**
    * current velocity with respect unit vector
    */
    Vector unit_v_air;

    /**
    * current mach
    */
    double mach;

    /**
    * current compressible dynamic pressure (Pa)
    */
    double dynamic_pressure;
};


class SingleStageAerodynamics
{

    static constexpr double AOA_THRESHOLD = 1e-6;

    double CD0;

    double CL_alpha;

    double CM_alpha;

    double CM_alpha_dot;

    double K;

    double ref_area;

    double ref_length;

    double stall_angle;

    double sin_stall_angle;

    double sin_zero_lift_angle;

    double CL_max;

    double constant_term;

    double CM_max;

    SingleStageRocket& rocket;

    void compute_aero_values();

    double get_parasitic_drag_from_mach(double mach);

    double get_angle_of_attack();

    struct aero_coef
    {
        double CL = 0.0;
        double CD = 0.0;;
        double CM = 0.0;;
    };

    aero_coef get_aero_coef(double sAoA);

public:

    AeroValues aero_values;

    Vector force;

    Vector moment;

    SingleStageAerodynamics(SingleStageRocket& r);

    void set_coef(double* coef);

    void update();

};
