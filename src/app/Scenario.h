#pragma once

#include <string>

class Scenario {

public:

    struct start {

        struct time {
            double JD2000;
            long unix_timestamp;
        };

        struct location {
            double latitude;
            double longitude;
            double altitude;
            double ECEF[3];
        };

        struct orientation {
            double ECEF[3][3];
        };

    };

    enum SCENARIO_TYPE {
        LAUNCH, FLIGHT, ORBIT, RISK_ANALYSIS
    };

    SCENARIO_TYPE type;

    Scenario();

    void load(const std::string& fn);

    void set_time(int year, int month, int day, int hour, int minute, double sec);

    void set_location(double latitude, double longitude, double altitude);

    void run();

};
