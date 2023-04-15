#include "../include/SingleStageSimulation.h"

#include "../../common/include/util.h"
#include <exception>
#include <iostream>

#include "../include/Atmosphere.h"

#include "../../../lib/tinyxml/tinyxml2.h"

SingleStageSimulation::SingleStageSimulation() {}
SingleStageSimulation::~SingleStageSimulation() {}

void SingleStageSimulation::load(std::string fn)
{
    tinyxml2::XMLDocument simDocument;
    auto err = simDocument.LoadFile(fn.c_str());
    if(err != tinyxml2::XML_SUCCESS) {
        //Could not load file. Handle appropriately.
        throw std::invalid_argument("Could not load file.");
    }

    auto* root = simDocument.RootElement();
    if(!root) { return; }

    auto* RocketFileElement = root->FirstChildElement("Rocket");
    const char* rocket_fn = RocketFileElement->Attribute("File");

    if(!rocket_fn){ throw std::invalid_argument("No Rocket File."); }

    auto* AtmosphereElement = root->FirstChildElement("Atmosphere");
    if(AtmosphereElement)
    {
        double ground_altitude = AtmosphereElement->FirstChildElement("Altitude")->DoubleText();
        double ground_temperature = AtmosphereElement->FirstChildElement("Temperature")->DoubleText();
        double ground_pressure = AtmosphereElement->FirstChildElement("Pressure")->DoubleText();
        double lapse_rate = AtmosphereElement->FirstChildElement("LapseRate")->DoubleText();
        this->atmosphere = std::make_unique<AtmosphereTable>(ground_altitude,ground_pressure,ground_temperature,lapse_rate);
    }
    else
    {
        this->atmosphere = std::make_unique<Atmosphere>();
    }

    this->rocket.reset(new SingleStageRocket(this->atmosphere.get()));

    this->rocket->load(rocket_fn);
}

void SingleStageSimulation::run(std::string fn, const bool debug)
{
    FILE* output = fopen(fn.c_str(),"w");

    if(!output)
    {
        throw std::invalid_argument("could not open file.");
    }

    fprintf(output,"% 12.9f % 12.9f % 9.5f\n", launch.latitude, launch.longitude, launch.altitude);

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

    this->rocket->init(this->launch.pitch_angle,this->launch.heading);

    double dt = 1.0/512.0;
    double time = 0;
    double time_record = 0;

    while(time < 10000)
    {
        this->rocket->step(time,dt);

        if(time > time_record)
        {
            if(std::isnan(this->rocket->state.position.z) || this->rocket->state.position.z < -0.5) {
                break;
            }

            this->rocket->state.CS.gram_schmidt_orthogonalize();

            const double* pos = this->rocket->state.position.data;
            const double* q = this->rocket->state.CS.data;
            fprintf(output,OUTPUT_FORMAT.c_str(),
                    time, pos[0], pos[1], pos[2], q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], this->rocket->inertia.mass);

            time_record += this->record.t_interval;

            if(debug)
            {
                debug_output << time << " ";
                const auto& filtered_state = this->rocket->gnc.navigation->get_estimated_state(*this->rocket, time);
                char buf[150];
                sprintf(buf,POS_FORMAT.c_str(), filtered_state.position[0],filtered_state.position[1],filtered_state.position[2]);
                debug_output << buf;
                // auto& CS = filtered_state.CS;
                auto& CS = this->rocket->state.CS;
                sprintf(buf,CS_FORMAT.c_str(),CS.data[0],CS.data[1],CS.data[2],CS.data[3],CS.data[4],CS.data[5],CS.data[6],CS.data[7],CS.data[8]);
                debug_output << buf << std::endl;
            }
        }

        std::cout << "\r" << time << std::flush;
    }

    fclose(output);
}
