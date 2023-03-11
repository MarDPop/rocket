#include "../include/GNC.h"

#include "../include/SingleStageRocket.h"

GNC::GNC(const SingleStageRocket& _rocket) : rocket(_rocket) {}

GNC::~GNC() {}

void GNC::update(double time)
{
    auto estimated_state = this->navigation->get_estimated_state(this->rocket, time);

    auto desired_state = this->guidance->get_commanded_state(estimated_state, time);

    this->control->get_outputs(desired_state, estimated_state, time);
}


void SingleStageControl::set_controller_terms(double P_angle, double P_velocity, double C_velocity ){
    this->K1 = P_angle;
    this->K2 = P_velocity;
    this->C2 = C_velocity;
}


void SingleStageControl::set_aero_coef(double dCL, double dCD, double dCM, double fin_COP_z, double fin_COP_d){
    this->dCLdTheta = dCL; // remember that these already have fin area "built in"
    this->dCDdTheta = dCD;
    this->dCMdTheta = dCM;
    this->z = fin_COP_z;
    this->d = fin_COP_d;
    this->const_axial_term = dCL*fin_COP_d;
    this->const_planer_term = dCM - (z - this->rocket->inertia.COG)*dCL; // remember z should be negative distance from nose
}


void SingleStageControl::set_chute(double area_drogue, double area_deployed, double CD_drogue, double CD_deployed, double chord_length, double deployment_time) {
    this->chute.area_drogue = area_drogue;
    this->chute.area_deployed = area_deployed;
    this->chute.CD_drogue = CD_drogue;
    this->chute.CD_deployed = CD_deployed;
    this->chute.chord_length = chord_length;
    this->chute.deployment_time = deployment_time;
}

void SingleStageControl::update_commands() {
    // essentially straight up
    //if( fabs(this->CS_measured.axis.z.z()) > 0.99999)
    const auto& CS = this->filter->get_computed_CS();
    Vector arm_inertial(-CS.axis.z.y,CS.axis.z.x,0); // rocket.z cross z to get correct sign
    Vector commanded_angular_rate = arm_inertial*this->K1;
    Vector angular_err = commanded_angular_rate - this->filter->get_computed_angular_rate();
    Vector commanded_torque = angular_err*this->K2 - this->filter->get_computed_angular_rate()*this->C2;
    commanded_torque = CS * commanded_torque;

    this->command_fins(commanded_torque);
}


void SingleStageControl::chute_dynamics(double time)
{
    this->chute.frac_deployed = (time - this->chute_deployed_time)/this->chute.deployment_time;
    double CDA;
    if(this->chute.frac_deployed > 1) {
        CDA = this->chute.CD_deployed*this->chute.area_deployed;
    } else {
        double CD = this->chute.CD_drogue + (this->chute.CD_deployed - this->chute.CD_drogue)*this->chute.frac_deployed;
        double A = this->chute.area_drogue + (this->chute.area_deployed - this->chute.area_drogue)*this->chute.frac_deployed;
        CDA = CD*A;
    }

    // chute is more complicated than this, but for now assume just a drag force

    this->dForce = this->rocket->aerodynamics->aero_values.unit_v_air * (-CDA*this->rocket->aerodynamics->aero_values.dynamic_pressure);
    // assume arm is nose

    //Vector::cross((this->rocket->state.CS.axis.z * this->rocket->inertia.COG),this->dForce,this->dMoment);
}


void SingleStageControl::update(double time)
{

    this->sensors->update(*this->rocket, time);

    this->filter->update(*this->sensors, time);

    if(this->chute_deployed)
    {
        return;
    }

    if(this->sensors->get_measured_dynamic_pressure() < 1e-3)
    {
        return;
    }

    double ascent_rate = this->filter->get_computed_velocity()[2];

    if(ascent_rate < -0.5)
    {
        this->rocket->parachute->deploy();
        return;
    }

    if(ascent_rate < 10)
    {
        this->dMoment.zero();
        this->dForce.zero();
        this->time_old = time;
        return;
    }

    this->deflect_fins(time);
    this->update_force();
    this->update_commands();
    this->time_old = time;
}

SingleStageControl_3::SingleStageControl_3() : SingleStageControl(3) {}

void SingleStageControl_3::set_aero_coef(double dCL, double dCD, double dCM, double fin_z, double fin_COP_d)
{
    SingleStageControl::set_aero_coef(dCL, dCD, dCM, fin_z, fin_COP_d);
    Axis A;
    for(unsigned i = 0; i < 3; i++)
    {
        A.data[i] = this->const_planer_term*this->fins[i].span.x;
        A.data[i+3] = this->const_planer_term*this->fins[i].span.y;
        A.data[i+6] = this->const_axial_term;
    }
    this->solve3 = A.get_inverse();
}

void SingleStageControl_3::command_fins(const Vector& commanded_torque) {

    Vector command_torque_scaled = commanded_torque * (1.0/this->sensors->get_measured_dynamic_pressure());

    Vector angles = this->solve3*command_torque_scaled;
    this->fins[0].commanded_deflection = angles.data[0];
    this->fins[1].commanded_deflection = angles.data[1];
    this->fins[2].commanded_deflection = angles.data[2];
}

SingleStageControl_4::SingleStageControl_4() : SingleStageControl(4) {}

void SingleStageControl_4::command_fins(const Vector& commanded_torque) {

    std::array<double,4> beta;
    for(unsigned i = 0; i < 4; i++)
    {
        beta[i] = 1 + this->fins[i].span.x + this->fins[i].span.y;
    }
    auto num = (commanded_torque.x + commanded_torque.y)/this->const_planer_term + commanded_torque.z/this->const_axial_term;
    auto den = this->sensors->get_measured_dynamic_pressure()*std::accumulate(beta.begin(),beta.end(),0);

    auto lambda = num/den;
    this->fins[0].commanded_deflection = lambda*beta[0];
    this->fins[1].commanded_deflection = lambda*beta[1];
    this->fins[2].commanded_deflection = lambda*beta[2];
    this->fins[3].commanded_deflection = lambda*beta[3];
}
