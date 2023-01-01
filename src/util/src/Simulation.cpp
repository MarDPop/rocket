#include "../include/Simulation.h"

#include <fstream>
#include <stdexcept>
#include <ctime>

SingleSimulation::SingleSimulation() {
}

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
    this->type = static_cast<SIMULATION_TYPE>(std::stoi(line));
    std::getline(file, line);
    this->unix_ms = std::stol(line);
    this->JD2000 = static_cast<double>(this->unix_ms - Time::J2000_UNIX)/static_cast<double>(Time::JULIAN_DAY);
    std::getline(file, line);
    this->lla.latitude = std::stod(line);
    std::getline(file, line);
    this->lla.longitude = std::stod(line);
    std::getline(file, line);
    this->lla.altitude = std::stod(line);

    this->position_ecef = Ellipsoid::geodetic2ecef(this->lla);



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

    this->unix_ms = static_cast<unsigned long>((seconds_unix + sec_frac)*1000);
}

void SingleSimulation::set_location(Geodetic& lla) {
    this->lla = lla;
    this->position_ecef = Ellipsoid::geodetic2ecef(lla);
}

void SingleSimulation::run() {

}
