#pragma once

#include "../common/include/Geodesy.h"
#include <string>
#include <memory>
#include "../6DOF/include/Vehicle.h"

class SingleSimulation {

    std::unique_ptr<Vehicle> vehicle;

    /* starting */
    double JD2000;
    unsigned long unix_ms;
    Geodetic lla;
    Vector position_ecef;
    Axis orientation_ecef;

public:

    enum SIMULATION_TYPE {
        LAUNCH, FLIGHT, ORBIT, RISK_ANALYSIS
    };

    SIMULATION_TYPE type;

    SingleSimulation();

    void load(const std::string& fn);

    void set_gmt(int year, int month, int day, int hour, int minute, double sec, double time_zone);

    void set_location(Geodetic& lla);

    void run();

};
