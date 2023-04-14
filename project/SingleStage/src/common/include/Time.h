#pragma once

class Time {

    int mjd;

    double sec;

public:

    static constexpr int J2000_UNIX = 946728000;
    static constexpr int JULIAN_DAY = 86400;

    Time(){}
    virtual ~Time(){}

    Time(int MJD, double day_sec) : mjd(MJD), sec(day_sec) {}

};
