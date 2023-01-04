#pragma once

#include <vector>

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

class SingleStageRocket;

struct Fin {
    double deflection = 0; // first fin on + x axis, going counter clockwise
    double commanded_deflection = 0; // first fin on + x axis, going counter clockwise
    Vector span; // vector of the direction of the span of all fins
    Vector lift; // vector of the direction of the lift of all fins
};

struct Chute {
    double area_drogue;
    double CD_drogue;
    double area_deployed;
    double CD_deployed;
    double deployment_time = 1;
    double chord_length;
    double frac_deployed;
};

class SingleStageControl {

protected:
    std::vector<Fin> fins;

    Chute chute;

    double z;

    double d; // distance along span vector of  on Center of pressure

    double dCMdTheta;

    double dCDdTheta;

    double dCLdTheta;

    double const_axial_term;

    double const_planer_term;

    double max_theta = 0.1; // rad

    double slew_rate = 0.5; // rad/s

    double K1;

    double K2;

    double C2;

    double time_old;

    Sensors sensors;

    double chute_deployment_time;

    bool chute_deployed;

    SingleStageRocket& rocket;

public:

    const unsigned NFINS;

    Vector dForce;

    Vector dMoment;

    SingleStageControl(SingleStageRocket& r, unsigned int N);

    void set_system_limits(double slew_limit, double angle_limit);

    void set_controller_terms(double P_angle, double P_velocity, double C_velocity );

    virtual void set_aero_coef(double dCL, double dCD, double dCM, double fin_z, double fin_COP_d);

    void set_chute(double area_drogue, double area_deployed, double CD_drogue, double CD_deployed, double chord_length, double deployment_time);

    void reset();

    void update(double time);

    void deflect_fins(double time);

    void update_force();

    void update_commands();

    void chute_dynamics(double time);

    virtual void command_fins(const Vector& commanded_torque) = 0;

};

class SingleStageControl_3 : public virtual SingleStageControl {

    Axis solve3;

public:

    SingleStageControl_3(SingleStageRocket& r);

    void set_aero_coef(double dCL, double dCD, double dCM, double fin_z, double fin_COP_d) override;

    void command_fins(const Vector& commanded_torque) override;

};

class SingleStageControl_4 : public virtual SingleStageControl {

public:

    SingleStageControl_4(SingleStageRocket& r);

    void command_fins(const Vector& commanded_torque) override;
};