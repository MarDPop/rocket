#include "../include/SingleStageControl.h"

SingleStageControl::SingleStageControl(SingleStageRocket& r, unsigned int N) : fins(N), rocket(r), NFINS(N) {

    for(unsigned int i = 0; i < NFINS;i++){
        this->fins[i].span.zero();
        this->fins[i].deflection = 0;
    }

    this->fins[0].span.x(1);
    double dtheta = 6.283185307179586476925286766559/NFINS;
    for(unsigned int i = 1; i < NFINS; i++){
        this->fins[i].span.data[0] = cos(i*dtheta);
        this->fins[i].span.data[1] = sin(i*dtheta);
    }

    this->chute_deployed = false;
}

void SingleStageControl::set_system_limits(double slew_limit, double angle_limit){
    this->max_theta = angle_limit;
    this->slew_rate = slew_limit;
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
    this->const_planer_term = dCM - (z - rocket.COG)*dCL; // remember z should be negative distance from nose
}

void SingleStageControl::set_chute(double area_drogue, double area_deployed, double CD_drogue, double CD_deployed, double chord_length, double deployment_time) {
    this->chute.area_drogue = area_drogue;
    this->chute.area_deployed = area_deployed;
    this->chute.CD_drogue = CD_drogue;
    this->chute.CD_deployed = CD_deployed;
    this->chute.chord_length = chord_length;
    this->chute.deployment_time = deployment_time;
}

void SingleStageControl::reset() {
    this->chute_deployed = false;
}

void SingleStageControl::deflect_fins(double time) {
    double max_angle = this->slew_rate*(time - this->time_old);
    for(auto& fin : this->fins) {
        double delta = fin.commanded_deflection - fin.deflection;
        if(fabs(delta) > max_angle) {
            fin.deflection += std::copysign(max_angle,delta);
        } else {
            fin.deflection = fin.commanded_deflection;
        }

        fin.deflection = std::copysign(std::min(fabs(fin.deflection),this->max_theta),fin.deflection);
    }
}

void SingleStageControl::update_force() {
    this->dMoment.zero();
    this->dForce.zero();

    double axial_term = this->dCLdTheta*this->d;
    double planar_term = this->dCMdTheta - (this->z - rocket.COG)*this->dCLdTheta;

    for(auto& fin : this->fins) {
        double tmp = planar_term*fin.deflection;

        this->dMoment.data[0] += fin.span.data[0]*tmp;
        this->dMoment.data[1] += fin.span.data[1]*tmp;
        this->dMoment.data[2] += fin.span.data[2]*axial_term*fin.deflection;

        tmp = this->dCLdTheta*fin.deflection;

        this->dForce.data[0] += fin.lift.data[0]*tmp;
        this->dForce.data[1] += fin.lift.data[1]*tmp;
        this->dForce.data[2] -= this->dCDdTheta*fin.deflection; // simply linear approximation for small angles
    }
    this->dMoment *= this->sensors.get_computed_dynamic_pressure();
    this->dForce *= this->sensors.get_computed_dynamic_pressure();
    // remember currently in body frame
    this->dMoment = rocket.CS.transpose_mult(this->dMoment);
    this->dForce = rocket.CS.transpose_mult(this->dForce);
}


void SingleStageControl::update_commands() {
    // essentially straight up
    //if( fabs(this->CS_measured.axis.z.z()) > 0.99999)

    Vector arm_inertial(-this->sensors.CS.axis.z.y(),this->sensors.CS.axis.z.x(),0); // rocket.z cross z to get correct sign
    Vector commanded_angular_rate = arm_inertial*this->K1;
    Vector angular_err = commanded_angular_rate - this->sensors.get_computed_angular_rate();
    Vector commanded_torque = angular_err*this->K2 - this->sensors.get_computed_angular_rate()*this->C2;
    commanded_torque = this->sensors.get_computed_CS() * commanded_torque;

    this->command_fins(commanded_torque);
}

void SingleStageControl::chute_dynamics(double time) {
    this->chute.frac_deployed = (time - this->chute_deployment_time)/this->chute.deployment_time;
    double CDA;
    if(this->chute.frac_deployed > 1) {
        CDA = this->chute.CD_deployed*this->chute.area_deployed;
    } else {
        double CD = this->chute.CD_drogue + (this->chute.CD_deployed - this->chute.CD_drogue)*this->chute.frac_deployed;
        double A = this->chute.area_drogue + (this->chute.area_deployed - this->chute.area_drogue)*this->chute.frac_deployed;
        CDA = CD*A;
    }

    // chute is more complicated than this, but for now assume just a drag force

    this->dForce = this->rocket.air.unit_v_air * (CDA*this->sensors.get_real_dynamic_pressure());
    // assume arm is nose
    Vector::cross((rocket.CS.axis.z*-rocket.COG),this->dForce,this->dMoment);
}

void SingleStageControl::update(double time) {

    this->sensors.compute_quantities(this.rocket, time);

    if(this->chute_deployed) {
        this->chute_dynamics(time);
        return;
    }

    if(this->sensors.get_measured_dynamic_pressure() < 1e-3) {
        return;
    }

    if(this->sensors.get_computed_ascent_rate() < -0.5) {
        this->chute_deployed = true;
        this->chute_deployment_time = time;
        return;
    }

    if(this->sensors.get_computed_ascent_rate() < 10) {
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

SingleStageControl_3::SingleStageControl_3(SingleStageRocket& r) : SingleStageControl(r,3) {}

void SingleStageControl_3::set_aero_coef(double dCL, double dCD, double dCM, double fin_z, double fin_COP_d) {
    SingleStageControl::set_aero_coef(dCL, dCD, dCM, fin_z, fin_COP_d);
    Axis A;
    for(int i = 0; i < 3;i++){
        A.data[i] = this->const_planer_term*this->fins[i].span.x();
        A.data[i+3] = this->const_planer_term*this->fins[i].span.y();
        A.data[i+6] = this->const_axial_term;
    }
    this->solve3 = A.get_inverse();
}

void SingleStageControl_3::command_fins(const Vector& commanded_torque) {

    Vector command_torque_scaled = commanded_torque * (1.0/this->sensors.get_measured_dynamic_pressure());

    Vector angles = this->solve3*command_torque_scaled;
    this->fins[0].commanded_deflection = angles.data[0];
    this->fins[1].commanded_deflection = angles.data[1];
    this->fins[2].commanded_deflection = angles.data[2];
}

SingleStageControl_4::SingleStageControl_4(SingleStageRocket& r) : SingleStageControl(r,4) {}

void SingleStageControl_4::command_fins(const Vector& commanded_torque) {

    std::array<double,4> beta;
    for(unsigned int i = 0; i < NFINS;i++) {
        beta[i] = 1 + this->fins[i].span.x() + this->fins[i].span.y();
    }
    auto num = (commanded_torque.x() + commanded_torque.y())/this->const_planer_term + commanded_torque.z()/this->const_axial_term;
    auto den = this->sensors.get_measured_dynamic_pressure()*std::accumulate(beta.begin(),beta.end(),0);

    auto lambda = num/den;
    this->fins[0].commanded_deflection = lambda*beta[0];
    this->fins[1].commanded_deflection = lambda*beta[1];
    this->fins[2].commanded_deflection = lambda*beta[2];
    this->fins[3].commanded_deflection = lambda*beta[3];
}
