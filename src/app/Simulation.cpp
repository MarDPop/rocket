#include "Simulation.h"
#include "Geodesy.h"

#include <fstream>
#include <stdexcept>

void SingleSimulation::load(const std::string& fn) {

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

    std::string line;

    std::getline(file, line);
    this->type = std::stoi(line);
    std::getline(file, line);
    this->start.time.unix_timestamp = std::stol(line);

    while (std::getline(file, line)){

    }

    file.close();
}

void SingleSimulation::set_gmt(int year, int month, int day, int hour, int minute, double sec, double time_zone){

    struct tm tm;
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_mday = minute;
    tm.tm_sec = static_cast<int>(sec);

    double sec_frac = sec - tm.tm_sec;

    double seconds_unix = difftime(mktime(&tm),0);

    this->start.time.unix_timestamp = mktime(&tm);
}

void SingleSimulation::set_location(Geodetic& lla) {
    this->start.location.lla = lla;
    this->start.location.ECEF = Ellipsoid::geodetic2ecef(lla);
}

void SingleSimulation::run() {

}
