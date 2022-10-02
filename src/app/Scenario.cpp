#include "Scenario.h"

#include <fstream>
#include <stdexcept>

void Scenario::load(const std::string& fn) {

    if(fn.size() < 6) {
        throw std::invalid_argument("invalid filename.");
    }

    std::string ext = fn.substr(fn.size() - 6);
    if(ext.compare(".scen")) {
        throw std::invalid_argument("not a scenario file.");
    }

    std::ifstream file(fn);

    if(!file.is_open()) {
        throw std::invalid_argument("could not open file.");
    }

    for (std::string line; std::getline(filein, line); ){
        unsigned int idx_comment = 0;
        while(idx_comment < line.size()) {
            if(line[idx_comment] == '#'){
                break;
            }
            idx_comment++;
        }
        if(idx_comment == line.size()) {
            line = line.substr(0,idx_comment);
        }

        unsigned int idx_colon = 0;
        while(idx_colon < line.size()) {
            if(line[idx_colon] == ':'){
                break;
            }
            idx_colon++;
        }

        if(idx_colon == line.size()) {
            line = line.substr(0,idx_comment);
        }
    }

    file.close();
}

void Scenario::set_time(int year, int month, int day, int hour, int minute, double sec){

}

void Scenario::set_location(double latitude, double longitude, double altitude) {

}

void Scenario::run() {

}
