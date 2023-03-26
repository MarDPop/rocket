#include "../include/SingleStageSimulation.h"

#include "../../common/include/util.h"
#include <exception>
#include <iostream>

#include "../../../lib/tinyxml/tinyxml2.h"

SingleStageSimulation::SingleStageSimulation() {}
SingleStageSimulation::~SingleStageSimulation() {}

void SingleStageSimulation::load(std::string fn)
{
    XMLDocument simDocument;
    auto err = doc.LoadFile(fn);
    if(err != tinyxml2::XML_SUCCESS) {
        //Could not load file. Handle appropriately.
        return;
    }

    auto* root = simDocument.RootElement();
    if(!root) { return; }

    auto* RocketFileElement = root->FirstChildElement("Rocket");
    const char* rocket_fn = RocketFileElement->attribute("File");

    if(!rocket_fn){ return;}

    auto* AtmosphereElement = root->FirstChildElement("Atmosphere");
    if(AtmosphereElement)
    {
        double ground_altitude = AtmosphereElement->FirstChildElement("Altitude")->DoubleValue();
        double ground_temperature = AtmosphereElement->FirstChildElement("Temperature")->DoubleValue();
        double ground_pressure = AtmosphereElement->FirstChildElement("Pressure")->DoubleValue();
        double lapse_rate = AtmosphereElement->FirstChildElement("LapseRate")->DoubleValue();
        this->atmosphere = std::make_unique<SimpleAtmosphere>();
    }
    else
    {
        this->atmosphere = std::make_unique<Atmosphere>();
    }

    this->rocket = std::move(SingleStageRocket::load(rocket_fn,this->atmosphere.get()));
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
