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
