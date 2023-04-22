#include "../include/SingleStageSimulation.h"

#include "../../common/include/util.h"
#include <exception>
#include <iostream>

#include "../include/Environment.h"

#include "../../../lib/tinyxml/tinyxml2.h"

SingleStageSimulation::SingleStageSimulation() {}
SingleStageSimulation::~SingleStageSimulation() {}

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
                const auto& filtered_state = this->rocket->gnc.navigation.filter->get_computed_state();
                char buf[150];
                sprintf(buf,POS_FORMAT.c_str(), filtered_state.position[0],filtered_state.position[1],filtered_state.position[2]);
                debug_output << buf;
                sprintf(buf,POS_FORMAT.c_str(), filtered_state.velocity[0],filtered_state.velocity[1],filtered_state.velocity[2]);
                debug_output << buf;
                auto& CS = filtered_state.CS;
                // auto& CS = this->rocket->state.CS;
                sprintf(buf,CS_FORMAT.c_str(),CS.data[0],CS.data[1],CS.data[2],CS.data[3],CS.data[4],CS.data[5],CS.data[6],CS.data[7],CS.data[8]);
                debug_output << buf << std::endl;
            }
        }

        std::cout << "\r" << time << std::flush;
    }

    fclose(output);
}
