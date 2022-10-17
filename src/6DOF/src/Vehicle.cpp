#include "../include/Vehicle.h"

#include "../../common/include/util.h"

#include <stdexcept>
#include <iostream>

Vehicle::Vehicle() {}

Vehicle::~Vehicle() {}

void Vehicle::set_orientation(double* q) {
    double mag = 1.0/sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] *= mag;
    q[1] *= mag;
    q[2] *= mag;
    q[3] *= mag;
    Cartesian::q2rotm(q,this->inertial_CS.data);
}

void Vehicle::get_state_rate(std::array<double,14>& x, double t, std::array<double,14>& dx) {

    this->set_orientation(&x[6]);

    this->body.update(this,t);

    this->current_stage->set_mass(x[13]);

    this->current_stage->compute();

    dx[13] = this->current_stage->mdot;

    Vector Force_in_inertial = this->inertial_CS.transpose_mult(this->force);

    double x_inv = 1.0/x[13];
    for(int i = 0; i < 3; i++) {
        dx[i] = x[i+3];
        dx[i+3] = Force_in_inertial.data[i]*x_inv + this->body.gravity->acceleration[i];
    }

    // Integration of quaternion
    dx[6] = -0.5*Cartesian::dot(&x[10],&x[7]);
    dx[7] = 0.5*(x[10]*x[6] + x[12]*x[8] - x[11]*x[9]);
    dx[8] = 0.5*(x[10]*x[7] - x[12]*x[7] + x[10]*x[9]);
    dx[9] = 0.5*(x[10]*x[8] + x[11]*x[7] - x[10]*x[9]);

    // Integration of angular velocity
    this->compute_moment(&x[10],&dx[10]);
}

void Vehicle::compute_no_moment(double* w, double* dw) {
    dw[0] = 0;
    dw[1] = 0;
    dw[2] = 0;
}

void Vehicle::compute_symmetric_moment(double* w, double* dw) {
    dw[0] = this->moment.data[0]/this->inertia[0];
    dw[1] = this->moment.data[1]/this->inertia[1];
    dw[2] = this->moment.data[2]/this->inertia[2];
}

void Vehicle::compute_plane_moment(double* w, double* dw) {
    dw[1] = (this->moment.data[1] + this->inertia[4]*(w[2]*w[2] - w[0]*w[0]) + w[0]*w[2]*(this->inertia[0] - this->inertia[2]))/this->inertia[1];
    double y1 = this->moment.data[0] + w[1]*(w[2]*(this->inertia[1] - this->inertia[2]) + w[0]*this->inertia[4]);
    double y2 = this->moment.data[2] + w[1]*(w[0]*(this->inertia[0] - this->inertia[1]) - w[2]*this->inertia[4]);

    double det = 1/(this->inertia[0]*this->inertia[2] + this->inertia[4]*this->inertia[4]);

    dw[0] = (this->inertia[2]*y1 - this->inertia[4]*y2)*det;
    dw[2] = (this->inertia[0]*y2 - this->inertia[4]*y1)*det;
}

void Vehicle::compute_full_moment(double* w, double* dw){
    double Iw[3];
    Iw[0] = this->inertia[0]*w[0] + this->inertia[3]*w[1] + this->inertia[4]*w[2];
    Iw[1] = this->inertia[3]*w[0] + this->inertia[1]*w[1] + this->inertia[5]*w[2];
    Iw[2] = this->inertia[4]*w[0] + this->inertia[5]*w[1] + this->inertia[2]*w[2];

    double wxIw[3];
    Cartesian::cross(w,Iw,wxIw); // NOTE: Due to this call, x must be the this->state.x
    Iw[0] = this->moment.data[0] - wxIw[0];
    Iw[1] = this->moment.data[1] - wxIw[1];
    Iw[2] = this->moment.data[2] - wxIw[2];

    double A[9];
    A[0] = this->inertia[1]*this->inertia[2] - this->inertia[5]*this->inertia[5];
    A[1] = this->inertia[4]*this->inertia[5] - this->inertia[3]*this->inertia[2];
    A[2] = this->inertia[3]*this->inertia[5] - this->inertia[4]*this->inertia[1];

    A[3] = this->inertia[5]*this->inertia[4] - this->inertia[3]*this->inertia[2];
    A[4] = this->inertia[0]*this->inertia[2] - this->inertia[4]*this->inertia[4];
    A[5] = this->inertia[3]*this->inertia[3] - this->inertia[0]*this->inertia[5];

    A[6] = this->inertia[3]*this->inertia[5] - this->inertia[1]*this->inertia[4];
    A[7] = this->inertia[3]*this->inertia[4] - this->inertia[0]*this->inertia[5];
    A[8] = this->inertia[0]*this->inertia[1] - this->inertia[3]*this->inertia[3];

    double det = 1.0/(this->inertia[0]*A[0] + this->inertia[3]*A[3] + this->inertia[4]*A[6]);

    dw[0] = (A[0]*Iw[0] + A[1]*Iw[1] + A[2]*Iw[2])*det;
    dw[1] = (A[3]*Iw[0] + A[4]*Iw[1] + A[5]*Iw[2])*det;
    dw[2] = (A[6]*Iw[0] + A[7]*Iw[1] + A[8]*Iw[2])*det;
}

WindHistory::WindHistory(std::string fn) {

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

WindHistory::WindHistory(std::vector<double> t, std::vector<Vector> s) : times(t) , speed(s) {
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

SingleStageRocket::SingleStageRocket() : aero(*this) {
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
    std::vector<std::string> data;

    if(!std::getline(file, line)) {
        throw std::runtime_error("file length invalid");
    }
    while(line.size() == 0 || line[0] == '#') {
        std::getline(file, line);
    }
    data = util::split(line);

    if(data.size() < 4) {
        throw std::runtime_error("Not enough empty mass information: " + std::to_string(data.size()) + " < 4. Reminder: {mass,Ixx,Izz,COG}");
    }

    double m_empty = std::stod(data[0]);
    double I_empty[3] = {std::stod(data[1]),std::stod(data[2]),std::stod(data[3])};

    if(!std::getline(file, line)) {
        throw std::runtime_error("file length invalid");
    }
    while(line.size() == 0 || line[0] == '#') {
        std::getline(file, line);
    }
    data = util::split(line);

    if(data.size() < 4) {
        throw std::runtime_error("Not enough full mass information: " + std::to_string(data.size()) + " < 4. Reminder: {mass,Ixx,Izz,COG}");
    }

    double m_full = std::stod(data[0]);
    double I_full[3] = {std::stod(data[1]),std::stod(data[2]),std::stod(data[3])};

    this->set_mass(m_empty,m_full,I_empty,I_full);

    if(!std::getline(file, line)) {
        throw std::runtime_error("file length invalid");
    }
    while(line.size() == 0 || line[0] == '#') {
        std::getline(file, line);
    }
    data = util::split(line);

    if(data.size() < 4) {
        throw std::runtime_error("Not enough ground information: " + std::to_string(data.size()) + " < 4. Reminder: {altitude,pressure,temperature,lapse rate}");
    }

    this->set_ground(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]),std::stod(data[3]));

    if(!std::getline(file, line)) {
        throw std::runtime_error("file length invalid");
    }
    while(line.size() == 0 || line[0] == '#') {
        std::getline(file, line);
    }
    data = util::split(line);

    if(data.size() < 2) {
        throw std::runtime_error("Not enough launch information: " + std::to_string(data.size()) + " < 2. Reminder: {heading,angle, (optional) wind_file}");
    }

    this->set_launch(std::stod(data[0]),std::stod(data[1]));

    if(data.size() == 3 && data[2].compare("skip") != 0) {
        try {
            this->wind = std::make_unique<WindHistory>(data[2]);
        } catch(...) {
        }
    }

    if(!std::getline(file, line)) {
        throw std::runtime_error("file length invalid");
    }
    while(line.size() == 0 || line[0] == '#') {
        std::getline(file, line);
    }
    data = util::split(line);

    if(data.size() < 8) {
        throw std::runtime_error("Not enough aerodynamic coefficients: " + std::to_string(data.size()) + " < 7. Reminder: {CD0,CL_a,CM_a,CM_a_dot,K,area,length,stall}");
    }

    double coef[8];
    for(int i = 0; i < 8; i++) {
        coef[i] = std::stod(data[i]);
    }

    this->aero.set_coef(coef);

    if(!std::getline(file, line)) {
        throw std::runtime_error("file length invalid");
    }
    while(line.size() == 0 || line[0] == '#') {
        std::getline(file, line);
    }
    data = util::split(line);

    if(data.size() < 2) {
        throw std::runtime_error("Need to Set Thruster Points");
    }

    int nPoints = std::stoi(data[1]);

    for(int i = 0; i < nPoints; i++) {
        std::getline(file, line);
        data = util::split(line);
        if(data.size() < 3) {
            throw std::runtime_error("Not enough thruster data in row");
        }
        this->thruster.add_thrust_point(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]));
    }

    if(!std::getline(file, line)) {
        return;
    }
    while(line.size() == 0 || line[0] == '#') {
        std::getline(file, line);
    }
    data = util::split(line);

    if(data.size() > 0) {
        this->record.t_interval = std::stod(data[0]);
    }

    if(data.size() > 1) {
        // load control here
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
        this->CS.axis.z.x(stheta*sphi);
        this->CS.axis.z.y(stheta*cphi);
        this->CS.axis.z.z(ctheta);
        Cartesian::cross(this->CS.axis.x.data,this->CS.axis.z.data,this->CS.axis.y.data);
    }
    this->position.zero();
    this->velocity.zero();
    this->angular_velocity.zero();
    this->set_ground(0,101000,297,0);
    this->mass = this->mass_full;
    this->Ixx = this->I_empty[1] + this->dIdm[1]*(this->mass - this->mass_empty);
    this->Izz = this->I_empty[1] + this->dIdm[1]*(this->mass - this->mass_empty);
}

void SingleStageRocket::compute_acceleration(double time) {

    this->get_air_properties();

    if(this->wind){
        this->wind->set(time);
    }

    if(this->mass > this->mass_empty) {
        this->thruster.set(this->air_pressure);
        this->acceleration = this->CS.axis.z * this->thruster.thrust;
    }

    this->aero.update();

    this->acceleration += this->aero.force;
    this->acceleration *= (1.0/this->mass);
    this->acceleration.data[2] -= this->grav;

    Axis IcRT; // inertia in body, multiplied by CS
    int i = 0;
    while(i < 6) {
        IcRT.data[i] = this->Ixx * this->CS.data[i];
        i++;
    }
    while(i < 9) {
        IcRT.data[i] = this->Izz * this->CS.data[i];
        i++;
    }
    Axis I = this->CS.transpose_mult(IcRT);
    Vector Iw = I*this->angular_velocity;
    Vector wIw = this->angular_velocity.cross(Iw);
    Vector rhs = this->aero.moment - wIw;
    this->angular_acceleration = I.get_inverse() * rhs;
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

        this->compute_acceleration(time);

        p0 = this->position;
        v0 = this->velocity;
        a0 = this->acceleration;
        w0 = this->angular_velocity;
        t0 = this->angular_acceleration;
        M0 = this->CS;

        this->position += this->velocity*dt;
        this->velocity += this->acceleration*dt;

        //Vector w = M0.transpose_mult(this->angular_velocity*dt);
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
            double dm = this->mass - this->mass_empty;
            this->Ixx = this->I_empty[0] + this->dIdm[0]*dm;
            this->Izz = this->I_empty[1] + this->dIdm[1]*dm;
            this->COG = this->I_empty[2] + this->dIdm[2]*dm;
        }

        this->compute_acceleration(time);

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

            time_record += this->record.t_interval;

            if(this->position.z() < -0.5) {
                break;
            }

            if(!burnout && this->mass <= this->mass_empty) {
                std::cout << "Burnout time: " << time << std::endl;
                burnout = true;
            }
        }

        time += dt;
    }
}
