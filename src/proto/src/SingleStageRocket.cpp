#include "../include/SingleStageRocket.h"

#include "../../common/include/util.h"
#include <fstream>
#include <numeric>
#include <iostream>

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
