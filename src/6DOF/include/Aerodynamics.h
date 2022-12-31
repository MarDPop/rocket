#pragma once

#include "Action.h"
#include "Atmosphere.h"
#include "../../common/include/Table.h"

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

template <unsigned N_CONTROL >
class AerodynamicsTable : public Aerodynamics {

    // key is Mach, Reynolds, Pitch angle, Sideslip Angle, + control deflections
    // CFx, CFy, CFz, CMx, CMy, CMz
    NestedTable<4 + N_CONTROL, 6> table;

public:

    AerodynamicsTable(std::string fn);

};
