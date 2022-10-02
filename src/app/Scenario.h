#pragma once


class Scenario {

    struct start_time {
        double JD2000;
        long unix_timestamp;
    };

    struct start_location {
        double latitude;
        double longitude;
        double height_WGS84;
        double geoid_height;
    };

public:

    enum SCENARIO_TYPE {
        LAUNCH, FLIGHT, ORBIT, RISK_ANALYSIS
    };

    Scenario();


};
