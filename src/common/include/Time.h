#pragma once


class Time {

    int mjd;

    double sec;

public:

    Time(){}
    virtual ~Time(){}

    Time(int MJD, double day_sec) : mjd(MJD), sec(day_sec) {}

};
