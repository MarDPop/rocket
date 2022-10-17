#include "../include/Thruster.h"
#include "../../common/include/util.h"

#include <fstream>

void Thruster::update(double time) {}

SolidThruster_precomputed::SolidThruster_precomputed(const std::string& fn) {
    std::ifstream myfile(fn);
    this->times.clear();
    this->mass_rates.clear();
    this->p_exit.clear();
    this->v_exit.clear();
    if(myfile.is_open()){
        for( std::string line; getline( myfile, line ); ){
            auto row = util::split(line);
            times.push_back(std::stod(row[0]));
            mass_rates.push_back(std::stod(row[1]));
            p_exit.push_back(std::stod(row[2]));
            v_exit.push_back(std::stod(row[3]));
        }
        myfile.close();
    }
}

SolidThruster_precomputed::~SolidThruster_precomputed(){}

void SolidThruster_precomputed::update(double time) {

}
