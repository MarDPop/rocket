#pragma once

#include "../include/Sensors.h"
#include "../include/Filter.h"

#include <memory>
#include <array>

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

class SingleStageRocket;

class Control
{
protected:

    SingleStageRocket* rocket = nullptr;

    virtual void compute_output(double time);

public:

    std::unique_ptr<Filter> filter;

    std::unique_ptr<Sensors> sensors;

    virtual ~Control();

    inline void set_rocket(SingleStageRocket* rocket)
    {
        this->rocket = rocket;
    }

    void update(double time);
};

class FinControl : public virtual Control
{

    double K1;

    double K2;

    double C2;

    double time_old;

    /*  */

    void deflect_fins(double time);

    void update_force();

    void update_commands();

    void chute_dynamics(double time);

    virtual void command_fins(const Vector& commanded_torque) = 0;

public:

    const unsigned NFINS;

    SingleStageControl(unsigned N);

    void set_system_limits(double slew_limit, double angle_limit);

    void set_controller_terms(double P_angle, double P_velocity, double C_velocity );

    virtual void set_aero_coef(double dCL, double dCD, double dCM, double fin_z, double fin_COP_d);

    void set_chute(double area_drogue, double area_deployed, double CD_drogue, double CD_deployed, double chord_length, double deployment_time);

    void reset();

    void compute_output(double time) override;

};

class SingleStageControl_3 : public virtual FinControl {

    Axis solve3;

public:

    SingleStageControl_3();

    void set_aero_coef(double dCL, double dCD, double dCM, double fin_z, double fin_COP_d) override;

    void command_fins(const Vector& commanded_torque) override;

};

class SingleStageControl_4 : public virtual FinControl {

public:

    SingleStageControl_4();

    void command_fins(const Vector& commanded_torque) override;
};
