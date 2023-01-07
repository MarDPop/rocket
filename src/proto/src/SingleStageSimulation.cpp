#include "../include/SingleStageSimulation.h"

#include "../../common/include/util.h"
#include <exception>
#include <iostream>

SingleStageSimulation::SingleStageSimulation() {}
SingleStageSimulation::~SingleStageSimulation() {}

void SingleStageSimulation::load(std::string fn){

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

    this->rocket.set_mass(m_empty,m_full,I_empty,I_full);

    data = util::split(lines[2]);
    if(data.size() < 4) {
        throw std::runtime_error("Not enough ground information: " + std::to_string(data.size()) + " < 4. Reminder: {altitude,pressure,temperature,lapse rate}");
    }

    this->rocket.altitude_table.set_ground(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]),std::stod(data[3]), 9.806);

    data = util::split(lines[3]);

    if(lines[3].size() > 1 && data[2].compare("skip") != 0) {
        try {
            this->rocket.altitude_table.wind.load(lines[3]);
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

    this->rocket.aerodynamics.set_coef(coef);

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
        this->rocket.thruster.add_thrust_point(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]));
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
        this->rocket.control = std::make_unique<SingleStageControl_3>(this->rocket);
    } else if (NFINS == 4) {
        this->rocket.control = std::make_unique<SingleStageControl_4>(this->rocket);
    } else {
        throw std::runtime_error("Currently only 3 or 4 fins are supported");
    }

    this->rocket.control->set_aero_coef(std::stod(data[1]),std::stod(data[2]),std::stod(data[3]),std::stod(data[4]),std::stod(data[5]));

    data = util::split(lines[7 + nPoints]);
    if(data.size() < 5) {
        throw std::runtime_error("Not enough Fin Info: " + std::to_string(data.size()) + " < 5. Reminder: {K1,K2,C2,slew,limit}");
    }

    this->rocket.control->set_controller_terms(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]));
    this->rocket.control->set_system_limits(std::stod(data[3]),std::stod(data[4]));

    data = util::split(lines[8 + nPoints]);
    if(data.size() == 6) {
        this->rocket.control->set_chute(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]),std::stod(data[3]),std::stod(data[4]),std::stod(data[5]));
    } else {
        std::cout << "no chute modeled.\n";
    }
}

void print_out(SingleStageRocket& rocket, const char* fn) {

    FILE* file = fopen(fn,"w");

    if(!file) {
        throw std::invalid_argument("could not open file.");
    }

    fprintf(file,"39.94426809919236 -104.94474985717818 1606.0\n");

    int nLines = rocket.record.position.size();

    for(int i = 0; i < nLines; i++) {
        const double* pos = rocket.record.position[i].data;
        const double* q = rocket.record.orientation[i].data;
        const double m = rocket.record.mass[i];
        fprintf(file,"%7.2f % .6e % .6e % .6e % 10.8f % 10.8f % 10.8f % 10.8f % 10.8f % 10.8f % 10.8f % 10.8f % 10.8f % .6e\n",
                    i*rocket.record.t_interval, pos[0], pos[1], pos[2], q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], m);
    }

    fclose(file);
}

void SingleStageSimulation::run(const char* fn) {

    std::cout << "Running Simulation." << std::endl;

    this->rocket.init(this->launch_angle,this->launch_heading);

    this->rocket.launch(0.1);

    std::cout << "Printing." << std::endl;

    print_out(this->rocket, fn);
}