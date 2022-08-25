#pragma once


class Time {

    double sec;

    int mjd;

public:

    Time(){}
    virtual ~Time(){}

    Time(int MJD, double day_sec) : mjd(MJD), sec(day_sec) {}

};
