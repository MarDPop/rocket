#include "../include/SingleStageRocket.h"

#include "../../common/include/util.h"
#include <fstream>
#include <numeric>
#include <iostream>

void SingleStageThruster::add_thrust_point(double pressure, double thrust, double mass_rate) {

    if(this->pressures.size() > 0 && pressure > this->pressures.back()) {
        auto i = this->pressures.end();
        while(i != this->pressures.begin()) {
            if(pressure < *(i-1)) {
                break;
            }
            i--;
        }

        int idx = i - this->pressures.begin();

        this->pressures.insert(i,pressure);
        this->thrusts.insert(this->thrusts.begin()+idx,thrust);
        this->mass_rates.insert(this->mass_rates.begin()+idx,mass_rate);
    } else {
        this->pressures.push_back(pressure);
        this->thrusts.push_back(thrust);
        this->mass_rates.push_back(mass_rate);

        this->thrust = thrust;
        this->mass_rate = mass_rate;
    }

    this->idx_final = this->pressures.size()-1;

    this->is_constant = this->idx_final == 0;
}

void SingleStageThruster::reset() {
    this->idx = 0;
};

void SingleStageThruster::set(double pressure) {
    if(this->is_constant) {
        return;
    }

    while(this->idx < this->idx_final && pressure < this->pressures[this->idx + 1]) {
        this->idx++;

        double dp = 1.0 / (this->pressures[this->idx+1] - this->pressures[this->idx]);
        this->dT = (this->thrusts[this->idx+1] - this->thrusts[this->idx])*dp;
        this->dM = (this->mass_rates[this->idx+1] - this->mass_rates[this->idx])*dp;
    }

    double delta = pressure - this->pressures[this->idx];
    this->thrust = this->thrusts[this->idx] + this->dT*delta;
    this->mass_rate = this->mass_rates[this->idx] + this->dM*delta;
};

SingleStageAerodynamics::SingleStageAerodynamics(SingleStageRocket& r) : rocket(r) {

}

void SingleStageAerodynamics::set_coef(double* coef) {

    this->CD0 = coef[0]*coef[5];
    this->CL_alpha = coef[1]*coef[5];
    this->CM_alpha = coef[2]*coef[5]*coef[6];
    this->CM_alpha_dot = coef[3]*coef[5]*coef[6];
    this->K = coef[4] / coef[5];
    this->ref_area = coef[5];
    this->ref_length = coef[6];
    this->stall_angle = coef[7];
    this->CL_max = this->CL_alpha*this->stall_angle;
    this->constant_term = 1.0/sin(this->stall_angle);
    this->CM_max = this->CM_alpha*2*this->stall_angle;

}

void SingleStageAerodynamics::update() {
    Vector airspeed_vec = rocket.velocity - rocket.wind.wind;

    double v2 = airspeed_vec.dot(airspeed_vec);

    if(v2 < 1e-2){
        this->force.zero();
        this->moment.zero();
        return;
    }

    double airspeed = sqrt(v2);

    double mach = airspeed * rocket.sound_speed_inv;

    double CD0_compressible;

    if (mach < 0.5) {
        CD0_compressible = this->CD0;
    } else {
        if(mach > 1) {
            CD0_compressible = this->CD0 + this->CD0/mach;
        } else {
            CD0_compressible = this->CD0*(mach - 0.5)*2.0;
        }
    }

    Vector unit_v = airspeed_vec * (1.0/airspeed);

    double proj = rocket.CS.axis.z.dot(unit_v);

    v2 *= rocket.air_density*0.5;

    this->moment = rocket.angular_velocity*(this->CM_alpha_dot*rocket.air_density);

    if (proj > 0.99998) {
        this->force = unit_v*(CD0_compressible*-v2);
        return;
    }

    double AoA = (proj > 0.9) ? sqrt(2 - 2*proj) : acos(proj);

    Vector arm = unit_v.cross(rocket.CS.axis.z);

    this->moment += arm*(this->CM_alpha*v2);

    double CL,CD;
    if(AoA > this->stall_angle) {
        CD = CD0_compressible + this->K*this->CL_max*this->CL_max*arm.norm()*this->constant_term;
        if(AoA > 2*this->stall_angle) {
            this->force = unit_v*(-CD*v2);
            return;
        } else {
            double frac = 1.0/(5*(AoA - this->stall_angle) + 1.0);
            CL = frac*this->CL_max;
        }
    } else {
        CL = this->CL_alpha*AoA;
        CD = CD0_compressible + this->K*CL*CL;
    }

    Vector lift = arm.cross(unit_v);
    double v = lift.norm();
    if(v > 1e-8) {
        CL /= v;
    }

    this->force = (lift*CL - unit_v*CD)*v2;
}

WindHistory::WindHistory() {
    this->wind.zero();
}

void WindHistory::load(std::string fn) {

    std::ifstream file(fn);

    if(!file.is_open()) {
        throw std::invalid_argument("could not open file.");
    }

    std::string line;

    while(std::getline(file, line)) {

        std::vector<std::string> data = util::split(line);

        this->times.push_back(std::stod(data[0]));
        this->speed.emplace_back(std::stod(data[1]),std::stod(data[2]),std::stod(data[3]));
    }

    file.close();

    this->reset();

    this->constant = this->times.size() < 2;
    if(this->constant){
        if(this->times.size() == 1) {
            this->wind = this->speed[0];
        } else {
            this->wind.zero();
        }
    }
}

SingleStageControl::SingleStageControl(SingleStageRocket& r, unsigned int N) : fins(N), rocket(r), NFINS(N) {

    for(unsigned int i = 0; i < NFINS;i++){
        this->fins[i].span.zero();
        this->fins[i].deflection = 0;
    }

    this->fins[0].span.x(1);
    double dtheta = 6.283185307179586476925286766559/NFINS;
    for(unsigned int i = 1; i < NFINS;i++){
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

void SingleStageControl::get_measured_quantities() {
    this->CS_measured = rocket.CS;
    this->angular_velocity_measured = rocket.angular_velocity;
    Vector airspeed = rocket.velocity - rocket.wind.wind;
    this->dynamic_pressure_measured = 0.5*rocket.air_density*airspeed.dot(airspeed);
    this->ascent_rate_measured = rocket.velocity.z();
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

    Vector airspeed = rocket.velocity - rocket.wind.wind;
    double dynamic_pressure = 0.5*rocket.air_density*airspeed.dot(airspeed);

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
    this->dMoment *= dynamic_pressure;
    this->dForce *= dynamic_pressure;
    // remember currently in body frame
    this->dMoment = rocket.CS.transpose_mult(this->dMoment);
    this->dForce = rocket.CS.transpose_mult(this->dForce);
}


void SingleStageControl::update_commands() {
    // essentially straight up
    //if( fabs(this->CS_measured.axis.z.z()) > 0.99999)

    Vector arm_inertial(-this->CS_measured.axis.z.y(),this->CS_measured.axis.z.x(),0); // rocket.z cross z to get correct sign
    Vector commanded_angular_rate = arm_inertial*this->K1;
    Vector angular_err = commanded_angular_rate - this->angular_velocity_measured;
    Vector commanded_torque = angular_err*this->K2 - this->angular_velocity_measured*this->C2;
    commanded_torque = this->CS_measured * commanded_torque;

    this->command_fins(commanded_torque);
}

void SingleStageControl::chute_dynamics(double time) {
    Vector airspeed = rocket.velocity - rocket.wind.wind;
    double dynamic_pressure = 0.5*rocket.air_density*airspeed.dot(airspeed);

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

    Vector unit_v = airspeed * (-1.0/airspeed.norm());

    this->dForce = unit_v * (CDA*dynamic_pressure);
    // assume arm is nose
    Vector::cross((rocket.CS.axis.z*-rocket.COG),this->dForce,this->dMoment);
}

void SingleStageControl::update(double time) {

    if(this->chute_deployed) {
        this->chute_dynamics(time);
        return;
    }

    this->get_measured_quantities();

    if(this->dynamic_pressure_measured < 1e-3) {
        return;
    }

    if(this->ascent_rate_measured < -0.5) {
        this->chute_deployed = true;
        this->chute_deployment_time = time;
        return;
    }

    if(this->ascent_rate_measured < 10) {
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

    Vector command_torque_scaled = commanded_torque * (1.0/this->dynamic_pressure_measured);

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
    auto den = this->dynamic_pressure_measured*std::accumulate(beta.begin(),beta.end(),0);

    auto lambda = num/den;
    this->fins[0].commanded_deflection = lambda*beta[0];
    this->fins[1].commanded_deflection = lambda*beta[1];
    this->fins[2].commanded_deflection = lambda*beta[2];
    this->fins[3].commanded_deflection = lambda*beta[3];
}

void WindHistory::load(std::vector<double> t, std::vector<Vector> s) {
    this->times = t;
    this->speed = s;
    this->reset();
    this->constant = t.size() == 1;
    if(this->constant){
        this->wind = this->speed[0];
    }
}

void WindHistory::reset() {
    this->titer = times.data();
    this->siter = speed.data();
    this->tend = this->titer + times.size() - 1;
    this->constant = times.size() == 1;
    if(!this->constant) {
        double dt = 1.0/(*(titer + 1) - *titer);
        this->dvdt = (*(siter + 1) - *siter)*dt;
    }
}

 void WindHistory::set(double time) {
    if(this->constant){
        return;
    }
    if(titer < tend && time > *(titer+1)) {
        titer++;
        siter++;
        if(titer == tend) {
            this->dvdt.zero();
            this->constant = true;
        } else {
            double dt = 1.0/(*(titer + 1) - *titer);
            this->dvdt = (*(siter + 1) - *siter)*dt;
        }
    }
    double dt = time - *titer;
    this->wind.data[0] = siter->data[0] + this->dvdt.data[0]*dt;
    this->wind.data[1] = siter->data[1] + this->dvdt.data[1]*dt;
    this->wind.data[2] = siter->data[2] + this->dvdt.data[2]*dt;
}

SingleStageRocket::SingleStageRocket(const std::string& fn) : aero(*this) {

    if(fn.size() < 8) {
        throw std::invalid_argument("invalid filename.");
    }

    std::string ext = fn.substr(fn.size() - 8);
    if(ext.compare(".srocket")) {
        throw std::invalid_argument("not a rocket file.");
    }

    std::ifstream file(fn);

    if(!file.is_open()) {
        throw std::invalid_argument("could not open file.");
    }

    std::string line;
    std::vector<std::string> lines;
    std::vector<std::string> data;

    while(std::getline(file, line)){
        if(line.size() == 0 || line[0] == '#'){
            continue;
        }
        lines.push_back(line);
    }

    if(lines.size() < 8) {
        throw std::runtime_error("file length invalid");
    }

    data = util::split(lines[0]);
    if(data.size() < 4) {
        throw std::runtime_error("Not enough empty mass information: " + std::to_string(data.size()) + " < 4. Reminder: {mass,Ixx,Izz,COG}");
    }

    double m_empty = std::stod(data[0]);
    double I_empty[3] = {std::stod(data[1]),std::stod(data[2]),std::stod(data[3])};

    data = util::split(lines[1]);
    if(data.size() < 4) {
        throw std::runtime_error("Not enough full mass information: " + std::to_string(data.size()) + " < 4. Reminder: {mass,Ixx,Izz,COG}");
    }

    double m_full = std::stod(data[0]);
    double I_full[3] = {std::stod(data[1]),std::stod(data[2]),std::stod(data[3])};

    this->set_mass(m_empty,m_full,I_empty,I_full);

    data = util::split(lines[2]);
    if(data.size() < 4) {
        throw std::runtime_error("Not enough ground information: " + std::to_string(data.size()) + " < 4. Reminder: {altitude,pressure,temperature,lapse rate}");
    }

    this->set_ground(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]),std::stod(data[3]));

    data = util::split(lines[3]);
    if(data.size() < 2) {
        throw std::runtime_error("Not enough launch information: " + std::to_string(data.size()) + " < 2. Reminder: {heading,angle, (optional) wind_file}");
    }

    this->set_launch(std::stod(data[0]),std::stod(data[1]));

    if(data.size() == 3 && data[2].compare("skip") != 0) {
        try {
            this->wind.load(data[2]);
        } catch(...) {}
    }

    data = util::split(lines[4]);
    if(data.size() < 8) {
        throw std::runtime_error("Not enough aerodynamic coefficients: " + std::to_string(data.size()) + " < 7. Reminder: {CD0,CL_a,CM_a,CM_a_dot,K,area,length,stall}");
    }

    double coef[8];
    for(int i = 0; i < 8; i++) {
        coef[i] = std::stod(data[i]);
    }

    this->aero.set_coef(coef);

    data = util::split(lines[5]);
    if(data.size() < 2) {
        throw std::runtime_error("Need to Set Thruster Points");
    }

    unsigned int nPoints = std::stoi(data[1]);

    for(unsigned int i = 0; i < nPoints; i++) {
        data = util::split(lines[6+i]);
        if(data.size() < 3) {
            throw std::runtime_error("Not enough thruster data in row");
        }
        this->thruster.add_thrust_point(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]));
    }

    if(lines.size() == 6 + nPoints){
        return;
    }

    if(lines.size() < 9 + nPoints){
        throw std::runtime_error("Not enough Fin Info");
    }

    data = util::split(lines[6 + nPoints]);
    if(data.size() < 6) {
        throw std::runtime_error("Not enough Fin Info: " + std::to_string(data.size()) + " < 6. Reminder: {NFINS,dSCL,dSCM,dSCD,COP_z,COP_radial}");
    }

    int NFINS = std::stoi(data[0]);
    if(NFINS == 3) {
        this->control = std::make_unique<SingleStageControl_3>(*this);
    } else if (NFINS == 4) {
        this->control = std::make_unique<SingleStageControl_4>(*this);
    } else {
        throw std::runtime_error("Currently only 3 or 4 fins are supported");
    }

    this->control->set_aero_coef(std::stod(data[1]),std::stod(data[2]),std::stod(data[3]),std::stod(data[4]),std::stod(data[5]));

    data = util::split(lines[7 + nPoints]);
    if(data.size() < 5) {
        throw std::runtime_error("Not enough Fin Info: " + std::to_string(data.size()) + " < 5. Reminder: {K1,K2,C2,slew,limit}");
    }

    this->control->set_controller_terms(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]));
    this->control->set_system_limits(std::stod(data[3]),std::stod(data[4]));

    data = util::split(lines[8 + nPoints]);
    if(data.size() == 6) {
        this->control->set_chute(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]),std::stod(data[3]),std::stod(data[4]),std::stod(data[5]));
    } else {
        std::cout << "no chute modeled.\n";
    }
}

void SingleStageRocket::init() {
    if(this->launch_angle == 0) {
        this->CS.identity();
    } else {
        double cphi = cos(this->launch_heading);
        double sphi = sin(this->launch_heading);
        double ctheta = cos(this->launch_angle);
        double stheta = sqrt(1 - ctheta*ctheta);
        this->CS.axis.x.x(cphi);
        this->CS.axis.x.y(-sphi);
        this->CS.axis.x.z(0);
        this->CS.axis.y.x(ctheta*sphi);
        this->CS.axis.y.y(ctheta*cphi);
        this->CS.axis.y.z(-stheta);
        this->CS.axis.z.x(stheta*sphi);
        this->CS.axis.z.y(stheta*cphi);
        this->CS.axis.z.z(ctheta);
        //Cartesian::cross(this->CS.axis.z.data,this->CS.axis.x.data,this->CS.axis.y.data);
    }
    this->position.zero();
    this->velocity.zero();
    this->angular_velocity.zero();
    this->mass = this->mass_full;
    this->Ixx = this->I_empty[1] + this->dIdm[1]*(this->mass - this->mass_empty);
    this->Izz = this->I_empty[1] + this->dIdm[1]*(this->mass - this->mass_empty);
}

void SingleStageRocket::compute_acceleration(double time) {

    this->get_air_properties();

    this->wind.set(time);

    this->aero.update();

    this->acceleration = this->aero.force;
    this->angular_acceleration = this->aero.moment;

    if(this->mass > this->mass_empty) {
        this->thruster.set(this->air_pressure);
        this->acceleration += this->CS.axis.z * this->thruster.thrust;
    }

    if(this->control){
        this->control->update(time);

        this->acceleration += this->control->dForce;
        this->angular_acceleration += this->control->dMoment;
    }

    this->acceleration *= (1.0/this->mass);
    this->acceleration.data[2] -= this->grav;

    Axis I_inertial = this->CS.get_transpose(); // might have to transpose CS
    int i = 0;
    for(; i < 6;i++) {
        I_inertial.data[i] *= this->Ixx;
    }
    for(; i < 9;i++) {
        I_inertial.data[i] *= this->Izz;
    }
    //Vector Iw = I_inertial * this->angular_velocity;
    //Vector rhs = this->angular_acceleration - this->angular_velocity.cross(Iw);
    // this->angular_acceleration = I_inertial.get_inverse() * rhs;
    this->angular_acceleration = I_inertial.get_inverse() * this->angular_acceleration;
}

void SingleStageRocket::set_launch(double launch_heading, double launch_angle) {
    this->launch_heading = launch_heading;
    this->launch_angle = launch_angle;
}

void SingleStageRocket::set_mass(double empty_mass, double full_mass, double I_empty[3], double I_full[3]) {
    this->mass_empty = empty_mass;
    this->mass_full = full_mass;
    this->I_empty[0] = I_empty[0]; // Ixx
    this->I_empty[1] = I_empty[1]; // Izz
    this->I_empty[2] = I_empty[2]; // COG
    double dm = full_mass - empty_mass;
    this->dIdm[0] = (I_full[0] - I_empty[0])/dm;
    this->dIdm[1] = (I_full[1] - I_empty[1])/dm;
    this->dIdm[2] = (I_full[2] - I_empty[2])/dm; // dCOG
    this->Ixx = I_full[0];
    this->Izz = I_full[1];
    this->COG = I_full[2];
}

void SingleStageRocket::get_inertia() {
    double dm = this->mass - this->mass_empty;
    this->Ixx = this->I_empty[0] + this->dIdm[0]*dm;
    this->Izz = this->I_empty[1] + this->dIdm[1]*dm;
    this->COG = this->I_empty[2] + this->dIdm[2]*dm;
}

void SingleStageRocket::set_ground(double ground_altitude, double ground_pressure ,double ground_temperature, double lapse_rate) {
    this->ground_altitude = ground_altitude;
    this->ground_pressure = ground_pressure;
    this->ground_temperature = ground_temperature;
    this->lapse_rate = lapse_rate;

    this->compute_atmosphere();
}

void SingleStageRocket::compute_atmosphere() {
    double pressure = this->ground_pressure;
    this->air_pressure_table.reserve(40000);
    this->air_density_table.reserve(40000);
    this->air_sound_speed_table.reserve(40000);
    this->grav_table.reserve(40000);
    double R0 = 6371000 + this->ground_altitude + 0.5;
    for(int i = 0; i < 40000; i++){
        double temperature = this->ground_temperature + i*lapse_rate;
        double density = pressure/(R_GAS*temperature);
        double r = 6371000.0/(R0 + i);
        double g = 9.806*r*r;
        this->air_pressure_table.push_back(pressure);
        this->air_density_table.push_back(density);
        this->air_sound_speed_table.push_back(1.0/sqrt(AIR_CONST*temperature));
        this->grav_table.push_back(g);

        pressure -= g*density; // dz = 1 meter
    }
}

void SingleStageRocket::get_air_properties() {
    int idx = static_cast<int>(this->position.z());
    if(idx > 39999) {
        idx = 39999;
    }
    if(idx < 0) {
        idx = 0;
    }
    this->air_density = this->air_density_table[idx];
    this->air_pressure = this->air_pressure_table[idx];
    this->sound_speed_inv = this->air_sound_speed_table[idx];
    this->grav = this->grav_table[idx];
}

void SingleStageRocket::launch(double dt) {
    double time = 0;
    double time_record = 0;
    double dt_half = dt*0.5;

    this->init();

    bool burnout = false;

    Axis mat,M0;
    Vector p0,v0,a0,w0,t0;
    while(time < 10000) {

        // Get initial state
        this->compute_acceleration(time);

        p0 = this->position;
        v0 = this->velocity;
        a0 = this->acceleration;
        w0 = this->angular_velocity;
        t0 = this->angular_acceleration;
        M0 = this->CS;

        // propagate to time + dt

        this->position += this->velocity*dt;
        this->velocity += this->acceleration*dt;

        Vector w = this->angular_velocity*dt;
        double angle = w.norm();
        if(angle > 1e-6) {
            w *= (1.0/angle);
            Cartesian::rotation_matrix_angle_axis(angle,w,mat);
            this->CS = mat*M0;
        }

        this->angular_velocity += this->angular_acceleration*dt;

        if(this->mass > this->mass_empty) {
            this->mass -= this->thruster.mass_rate*dt;
            this->get_inertia();
        }

        // recompute state rate at time + dt
        this->compute_acceleration(time + dt);

        this->position = p0 + (v0 + this->velocity)*dt_half;
        this->velocity = v0 + (a0 + this->acceleration)*dt_half;

        w = (w0 + this->angular_velocity)*dt_half;
        angle = w.norm();
        if(angle > 1e-6) {
            w *= (1.0/angle);
            Cartesian::rotation_matrix_angle_axis(angle,w,mat);
            this->CS = mat*M0;
        }

        this->angular_velocity = w0 + (t0 + this->angular_acceleration)*dt_half;

        if(time > time_record){
            this->record.position.push_back(this->position);
            this->record.orientation.push_back(this->CS);
            this->record.mass.push_back(this->mass);

            if(std::isnan(this->position.data[0])){
                break;
            }

            time_record += this->record.t_interval;

            if(this->position.z() < -0.5) {
                break;
            }

            if(!burnout && this->mass <= this->mass_empty) {
                burnout = true;
            }
        }

        time += dt;
    }
}
