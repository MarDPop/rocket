#pragma once

#include "Environment.h"

#include "../../common/include/Cartesian.h"
#include "../../common/include/Table.h"
#include "Action.h"
#include "Servo.h"

#include <memory>

using namespace Cartesian;

class SingleStageRocket;

struct AeroValues {
    /**
    * current air speed (m/s)
    */
    double airspeed;

    /**
    * current air velocity unit vector in body frame
    */
    Vector unit_v_air_body;

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
    * Force and moment in body frame
    */
    BodyAction _action;

    /**
    * available aero values such as mach
    */
    AeroValues _aero_values;

    /**
    * computes aero values from rocket reference (mach, dynamic pressure, etc)
    */
    void compute_aero_values();

    /**
    *
    */
    virtual void compute_forces();

public:

    inline const AeroValues& get_aero_values() const
    {
        return this->_aero_values;
    }

    Aerodynamics(SingleStageRocket& r);
    virtual ~Aerodynamics();

    /**
    * Returns a BodyAction in body space
    */
    const BodyAction& update();
};

class AerodynamicsBasicCoefficient : public virtual Aerodynamics
{
protected:
    static constexpr double AIRSPEED_THRESHOLD = 1e-2;

    static constexpr double AOA_THRESHOLD = 1e-6;

    double _CD0;

    double _CL_alpha;

    double _CM_alpha;

    double _CM_alpha_dot;

    double _K;

    double _ref_area;

    double _ref_length;

    double _stall_angle;

    double _CL_max;

    double _CM_max;

    double get_parasitic_drag_from_mach(double mach);

    double get_angle_of_attack();

    struct aero_coef
    {
        double CL = 0.0;
        double CD = 0.0;;
        double CM = 0.0;;
    };

    aero_coef get_aero_coef(double sAoA);

    virtual void compute_forces() override;

public:

    AerodynamicsBasicCoefficient(SingleStageRocket& r);
    virtual ~AerodynamicsBasicCoefficient();

    void set_coef(const std::array<double,9>& coef);
};

struct Fin
{
    /**
    * Servo associated with fin deflection
    */
    std::unique_ptr<Servo> servo;

    /**
    * span vector x (unit vector along span (normal to z) in body frame)
    */
    double span_x;

    /**
    * span vector y
    */
    double span_y;
};

class FinControlAero
{
protected:
    std::vector<Fin> fins;
public:
    const unsigned NUMBER_FINS;

    FinControlAero(unsigned NFINS);

    inline const Fin& get_fin(unsigned idx)
    {
        return this->fins[idx];
    }
};

class AerodynamicsFinCoefficient : public virtual AerodynamicsBasicCoefficient, public virtual FinControlAero
{
    double delta_z; // distance along z axis of span vectors from nose

    double span_d; // distance along span vector of Center of pressure

    double dCMdTheta; // change in moment ( on span vector )

    double dCDdTheta; // change in drag per angle ( on z axis, on center of pressure )

    double dCLdTheta; // change in lift ( on lift vector at center of pressure )

    double const_axial_term_lift;

    double const_axial_term_drag;

    double const_axial_term_moment;

    void compute_forces() override;

public:

    AerodynamicsFinCoefficient(SingleStageRocket& r, unsigned NFINS);
    ~AerodynamicsFinCoefficient();

    void set_fin_coef(const std::array<double,6>& coef);

};

class AerodynamicsTabulated : public virtual Aerodynamics, virtual FinControlAero
{
    std::vector<double> current_fin_deflection;

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


