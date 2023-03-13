#pragma once

#include "Atmosphere.h"

#include "../../common/include/Cartesian.h"
#include "../../common/include/Table.h"

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

class Aerodynamics
{
protected:

    /**
    * Rocket Reference
    */
    SingleStageRocket& rocket;

    /**
    * computes aero values from rocket reference (mach, dynamic pressure, etc)
    */
    void compute_aero_values();

public:

    /**
    * Force in inertial frame
    */
    Vector force;

    /**
    * Moment in inertial frame
    */
    Vector moment;

    /**
    * publically available aero values such as mach
    */
    AeroValues aero_values;


    Aerodynamics(SingleStageRocket& r);
    virtual ~Aerodynamics();


    virtual void update() = 0;
};

class SimpleAerodynamics : public virtual Aerodynamics
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

    SimpleAerodynamics(SingleStageRocket& r);

    void set_coef(double* coef);

    void update() override;
};

template <unsigned NUMBER_FINS>
class ControlledAerodynamics : public virtual Aerodynamics
{
protected:

    std::array<double, NUMBER_FINS> current_fin_deflection;

public:

    ControlledAerodynamics(SingleStageRocket& r);
    virtual ~ControlledAerodynamics();
};

template <unsigned NUMBER_FINS>
class FinCoefficientAerodynamics : public virtual ControlledAerodynamics<NUMBER_FINS>
{
    std::array<Vector, NUMBER_FINS> fin_span_vector;

    std::array<Vector, NUMBER_FINS> fin_lift_vector;

    double z; // distance along z axis of span vectors from nose

    double d; // distance along span vector of Center of pressure

    double dCMdTheta; // change in moment ( on span vector )

    double dCDdTheta; // change in drag per angle ( on z axis, on center of pressure )

    double dCLdTheta; // change in lift ( on lift vector at center of pressure )

public:

    FinCoefficientAerodynamics(SingleStageRocket& r);
    ~FinCoefficientAerodynamics();

    void set_aero_coef(double dCL, double dCD, double dCM, double fin_COP_z, double fin_COP_d);

};

template <unsigned NUMBER_FINS>
class TabulatedAerodynamics : public virtual ControlledAerodynamics<NUMBER_FINS>
{

    union aero_key
    {
        std::array<double,3> key;
        double alpha; // angle of attack from 0 - 45 deg
        double beta; // freestream azimuth angle from first fin to mid distance to next fin
        double mach; // mach
    };

    union fin_key
    {
        std::array<double,3> key;
        double alpha; // angle of attack from 0 - 45 deg
        double beta; // freestream azimuth angle from first fin to mid distance to next fin
        double deflection; // mach
    };

    NestedTable<3,6> fin_aero_delta_table;

    NestedTable<3,6> aero_table;

public:

    TabulatedAerodynamics(SingleStageRocket& r);
};


