#pragma once

#include "Action.h"
#include "Atmosphere.h"

class Vehicle;

class Aerodynamics : public virtual Action {
protected:

    Vehicle* vehicle;

    Air* air;

public:

    Aerodynamics() {}

    void set_vehicle(Vehicle* vehicle);

};

class AerodynamicsDragOnly : public Aerodynamics {

    const double CD_A;

public:

    AerodynamicsDragOnly(double CD, double A);
    ~AerodynamicsDragOnly();

    void update(double time) override;

};

class AerodynamicsBasic : public Aerodynamics {

    double CD0;

    double K;

    double CL_alpha;

    double CM_alpha;

    double stall_angle;

    double ref_area;

public:

    AerodynamicsBasic() {}

    void update(double time) override;

};

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
