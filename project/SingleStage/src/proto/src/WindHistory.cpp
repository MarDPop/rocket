#include "../include/WindHistory.h"

#include <iostream>
#include <fstream>
#include <string>
#include "../../common/include/util.h"

WindHistory::WindHistory() {
    this->wind.zero();
}

WindHistory::~WindHistory() {}

void WindHistory::load(std::string fn) {

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

 void WindHistory::set(double altitude, double time) {
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
