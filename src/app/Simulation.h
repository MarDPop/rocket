#pragma once

#include "Geodesy.h"
#include <string>

class SingleSimulation {

public:

    std::unique_ptr<Vehicle> vehicle;

    struct start {

        struct time {
            double JD2000;
            unsigned long unix_timestamp;
        };

        struct location {
            Geodetic lla;
            Vector ecef;
        };

        struct orientation {
            Axis ECEF;
        };

    };

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
