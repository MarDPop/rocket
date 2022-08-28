#include "../include/Atmosphere.h"
#include "../../common/include/util.h"

#include <fstream>
#include <string>

bool AtmosphereBasic::get_air(const std::array<double,3>& LLA, double time) {
    double factor = exp(-LLA[2]/scale_height);
    this->air.density = sea_density*factor;
    this->air.pressure = sea_pressure*factor;
    return true;
}

void AtmosphereTable::load(const char* fn) {
    std::ifstream file;
    file.open(fn);
    if(file.is_open()) {
        std::string line;
        while(getline(file,line)){
            auto row = util::split(line);
            if(row.size() < 6) {
                continue;
            }
            double h = std::stod(row[0]);
            std::array<double,5> values;
            for(int i = 0; i < 5;i++) {
                values[i] = std::stod(row[i+1]);
            }
            this->add(h,values);
        }
        file.close();
    }
}

void AtmosphereTable::add(double alt, const std::array< double, 5 >& values) {
    if(this->altitudes.isempty()){
        this->altitudes.push_back(alt);
        this->data.push_back(values);
        return;
    }

    if(alt > this->altitudes.back()) {
        this->altitudes.push_back(alt);
        this->data.push_back(values);
        this->delta.emplace_back();
        unsigned int n = this->altitudes.size() - 1;
        double dh = this->altitudes[n] - this->altitudes[n-1];
        auto& hi = this->data[n];
        auto& lo = this->data[n-1];
        auto& d = this->delta.back();
        for(int i = 0; i < 5; i++) {
            d[i] = (hi[i] - lo[i])/dh;
        }
        return;
    }

    std::array<double,5> d;
    if(alt < this->altitudes[0]) {
        this->altitudes.insert(this->altitudes.begin(),alt);
        this->data.insert(this->data.begin(),values);

        double dh = this->altitudes[1] - this->altitudes[0];
        auto& hi = this->data[1];
        auto& lo = this->data[0];
        for(int i = 0; i < 5; i++) {
            d[i] = (hi[i] - lo[i])/dh;
        }
        this->delta.insert(this->delta.begin(),d);
        return;
    }

    auto idx = util::bisection_search(this->altitudes.data(),alt,this->altitudes.size());
    this->altitudes.insert(this->altitudes.begin() + idx,alt);
    this->data.insert(this->data.begin() + idx,values);
    double dh = this->altitudes[idx+1] - this->altitudes[idx];
    auto& hi = this->data[idx+1];
    auto& lo = this->data[idx];
    for(int i = 0; i < 5; i++) {
        d[i] = (hi[i] - lo[i])/dh;
    }
    this->delta.insert(this->delta.begin()+idx,d);
}

bool AtmosphereUS1976::get_air(const std::array<double,3>& LLA, double time) {
     if(LLA[2] > 86) {
        return false;
    }

    if(LLA[0] <= 0) {
        this->air.density = this->data[0][1];
        this->air.pressure = this->data[0][0];
        this->air.speed_of_sound = this->data[0][2];
        this->air.temperature = this->data[0][3];
        this->air.dynamic_viscosity = this->data[0][4];
        return true;
    }

    int idx = int(LLA[2]);

    double factor = LLA[2] - idx;

    const double* row = this->data[idx];
    const double* d = this->delta[idx];

    for( int i = 0; i < 5; i++) {
        this->air.data[i] = row[i] + factor*d[i];
    }
    return true;
}
