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

    this->rocket.set_inertial_properties(m_empty,m_full,I_empty,I_full);

    data = util::split(lines[2]);
    if(data.size() < 4) {
        throw std::runtime_error("Not enough ground information: " + std::to_string(data.size()) + " < 4. Reminder: {altitude,pressure,temperature,lapse rate}");
    }

    this->rocket.altitude_table.set_ground(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]),std::stod(data[3]), 9.806);

    if(lines[3].size() > 5) {
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

    for(unsigned int i = 0; i < nPoints; i++)
    {
        data = util::split(lines[6+i]);
        if(data.size() < 3)
        {
            throw std::runtime_error("Not enough thruster data in row");
        }
        this->rocket.thruster.add_thrust_point(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]));
    }

    if(lines.size() == 6 + nPoints)
    {
        std::cout << "No control set \n";
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

    Control* control;
    if(NFINS == 3) {
        control = new SingleStageControl_3();
    } else if (NFINS == 4) {
        control = new SingleStageControl_4();
    } else {
        std::cout << "Currently only 3 or 4 fins are supported. No control will be run, but sensors will";

        control = new Control();

        control->sensors.reset(new Sensors());

        control->filter.reset(new FilterNone());
    }

    std::cout << "rocket with " << NFINS << " will be run\n";

    control->set_rocket(&this->rocket);

    this->rocket.control.reset(control);

    if(NFINS < 3 || NFINS > 4)
    {
        return;
    }

    SingleStageControl* c = dynamic_cast<SingleStageControl*>(control);

    c->set_aero_coef(std::stod(data[1]),std::stod(data[2]),std::stod(data[3]),std::stod(data[4]),std::stod(data[5]));

    data = util::split(lines[7 + nPoints]);
    if(data.size() < 5) {
        throw std::runtime_error("Not enough Fin Info: " + std::to_string(data.size()) + " < 5. Reminder: {K1,K2,C2,slew,limit}");
    }

    c->set_controller_terms(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]));
    c->set_system_limits(std::stod(data[3]),std::stod(data[4]));

    data = util::split(lines[8 + nPoints]);
    if(data.size() == 6) {
        c->set_chute(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]),std::stod(data[3]),std::stod(data[4]),std::stod(data[5]));
    } else {
        std::cout << "no chute modeled.\n";
    }
}

void SingleStageSimulation::run(std::string fn) {

    this->output = fopen(fn.c_str(),"w");

    if(!this->output)
    {
        throw std::invalid_argument("could not open file.");
    }

    fprintf(this->output,"39.94426809919236 -104.94474985717818 1606.0\n");

    std::cout << "Running Simulation." << std::endl;

    this->rocket.init(this->launch_angle,this->launch_heading);

    double dt = 1.0/256.0;
    double time = 0;
    double time_record = 0;

    while(time < 10000)
    {
        this->rocket.step(time,dt);

        if(time > time_record)
        {
            if(std::isnan(this->rocket.state.position.z) || this->rocket.state.position.z < -0.5) {
                break;
            }

            const double* pos = this->rocket.state.position.data;
            const double* q = this->rocket.state.CS.data;
            fprintf(this->output,"%7.2f % .6e % .6e % .6e % 10.8f % 10.8f % 10.8f % 10.8f % 10.8f % 10.8f % 10.8f % 10.8f % 10.8f % .6e\n",
                    time, pos[0], pos[1], pos[2], q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], this->rocket.inertia.mass);

            time_record += this->record.t_interval;

            this->rocket.state.CS.gram_schmidt_orthogonalize();
        }

        std::cout << "\r" << time << std::flush;
    }

    fclose(this->output);
}
