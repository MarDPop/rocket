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

    if(lines[5].size() < 6)
    {
        throw std::runtime_error("Need to Set Thruster File");
    }

    try
    {
        auto ext = util::get_extension(lines[5]);
        if(ext == "pthruster")
        {
            PressureThruster* thruster = new PressureThruster();
            thruster->load(lines[5]);
            this->rocket.thruster.reset(thruster);
        }
        else if (ext == "cthruster")
        {
            ComputedThruster* thruster = new ComputedThruster();
            thruster->load(lines[5]);
            this->rocket.thruster.reset(thruster);
            thruster->save("test/computed_thruster_file");
            thruster->set(10000,0);
            m_full = m_empty + thruster->mass;
            this->rocket.set_inertial_properties(m_empty,m_full,I_empty,I_full);
            // need to correct for mass
        }
        else if (ext == "sthruster")
        {
            SingleStageThruster* thruster = new SingleStageThruster();
            thruster->load(lines[5]);
            this->rocket.thruster.reset(thruster);
        }
    } catch(...) {}

    if(lines.size() == 6)
    {
        std::cout << "No control set \n";
        return;
    }

    if(lines.size() < 9)
    {
        throw std::runtime_error("Not enough Fin Info");
    }

    data = util::split(lines[6]);
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

    data = util::split(lines[7]);
    if(data.size() < 5) {
        throw std::runtime_error("Not enough Fin Info: " + std::to_string(data.size()) + " < 5. Reminder: {K1,K2,C2,slew,limit}");
    }

    c->set_controller_terms(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]));
    c->set_system_limits(std::stod(data[3]),std::stod(data[4]));

    data = util::split(lines[8]);
    if(data.size() == 6) {
        c->set_chute(std::stod(data[0]),std::stod(data[1]),std::stod(data[2]),std::stod(data[3]),std::stod(data[4]),std::stod(data[5]));
    } else {
        std::cout << "no chute modeled.\n";
    }
}

void SingleStageSimulation::run(std::string fn, const bool debug)
{

    this->output = fopen(fn.c_str(),"w");

    if(!this->output)
    {
        throw std::invalid_argument("could not open file.");
    }

    fprintf(this->output,"% 12.9f % 12.9f % 9.5f\n", launch.latitude, launch.longitude, launch.altitude);

    std::ofstream debug_output;
    if(debug)
    {
        std::cout << "Outputting Debug File" << std::endl;
        debug_output.open("debug.txt");
    }

    std::cout << "Running Simulation." << std::endl;

    const std::string CS_FORMAT = "% 14.12f % 14.12f % 14.12f % 14.12f % 14.12f % 14.12f % 14.12f % 14.12f % 14.12f ";
    const std::string POS_FORMAT = "% .6e % .6e % .6e ";
    const std::string OUTPUT_FORMAT = "%8.3f " + POS_FORMAT + CS_FORMAT + "% .6e\n";

    this->rocket.init(this->launch.pitch_angle,this->launch.heading);

    double dt = 1.0/512.0;
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

            this->rocket.state.CS.gram_schmidt_orthogonalize();

            const double* pos = this->rocket.state.position.data;
            const double* q = this->rocket.state.CS.data;
            fprintf(this->output,OUTPUT_FORMAT.c_str(),
                    time, pos[0], pos[1], pos[2], q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], this->rocket.inertia.mass);

            time_record += this->record.t_interval;

            if(debug)
            {
                debug_output << time << " ";
                auto computed_pos = this->rocket.control->filter->get_computed_position();
                auto CS = this->rocket.control->filter->get_computed_CS();
                char buf[150];
                sprintf(buf,POS_FORMAT.c_str(), computed_pos[0],computed_pos[1],computed_pos[2]);
                debug_output << buf;
                sprintf(buf,CS_FORMAT.c_str(),CS[0],CS[1],CS[2],CS[3],CS[4],CS[5],CS[6],CS[7],CS[8]);
                debug_output << buf << std::endl;
            }
        }

        std::cout << "\r" << time << std::flush;
    }

    fclose(this->output);
}
