#pragma once

#include <vector>

typedef long UNIX_TIMESTAMP;

typedef long GPS_TIMESTAMP;

class Time
{
    static std::vector<int> LEAP_SECONDS_TAI;

    static std::vector<int> LEAP_SECOND_MJD;

    double _day_sec; 

    int _mjdn;

public:

    static constexpr long SECONDS2NANOSECONDS = 1000000000L;

    static constexpr double JULIAN_DATE_J2000 = 2451545.0;

    static constexpr unsigned JDN_J2000 = 2451545;

    static constexpr unsigned JULIAN_DAY_SEC = 86400;

    static constexpr unsigned JULIAN_DAY_SEC_D = 86400.0;

    static constexpr long JULIAN_DAY_NANOSEC = JULIAN_DAY_SEC*SECONDS2NANOSECONDS;

    static constexpr double SEC_2_DAY_FRACTION = 1.0/86400.0;

    static constexpr double NANOSEC_2_DAY_FRACTION = 1.0/JULIAN_DAY_NANOSEC;

    static constexpr double MJD_JULIAN_DATE = 2400000.5;

    static constexpr double JULIAN_DATE_UNIX_EPOCH = 2440587.5;

    static constexpr int MJD_UNIX_EPOCH = 40587;

    static constexpr int UNIX_TAI_LEAPSECONDS = 10;

    static constexpr double HALF_DAY_SEC = 43200.0;

    inline Time(){}

    inline Time(int mjdn, double day_sec) : _mjdn(mjdn), _day_sec(day_sec) {}

    inline Time(UNIX_TIMESTAMP timestamp_ns)
    {
        long days = timestamp_ns / JULIAN_DAY_NANOSEC;
        long day_nanos = timestamp_ns - (days*JULIAN_DAY_NANOSEC);

        int leap_seconds_after_unix = get_TAI_leap_seconds(MJD_UNIX_EPOCH + static_cast<int>(days)); // TODO: see if need to subtract UNIX_TAI

        day_nanos -= leap_seconds_after_unix;
        if(day_nanos < 0)
        {
            day_nanos += JULIAN_DAY_NANOSEC;
            days--;
        }

        this->_mjdn = days + MJD_UNIX_EPOCH;
        this->_day_sec = day_nanos*NANOSEC_2_DAY_FRACTION;
    }

    inline int get_MJDN() const
    {
        return this->_mjdn;
    }

    inline double get_seconds_past_midnight() const
    {
        return this->_day_sec;
    }

    inline void operator+=(double sec)
    {
        this->_day_sec += sec;
        if(this->_day_sec > JULIAN_DAY_SEC_D) 
        {
            double days = this->_day_sec*SEC_2_DAY_FRACTION;
            int nDays = static_cast<int>(days);

            this->_mjdn += nDays;
            this->_day_sec = (days - nDays)*JULIAN_DAY_SEC_D;
        }
    }

    static int get_TAI_leap_seconds(int mjdn) 
    {
        static int _leap_idx = 0;
        if(mjdn > LEAP_SECOND_MJD.back())
        {
            return LEAP_SECONDS_TAI.back();
        }

        while(mjdn > LEAP_SECOND_MJD[_leap_idx + 1] && _leap_idx < LEAP_SECOND_MJD.size())
        {
            _leap_idx++;
        }

        while(mjdn < LEAP_SECOND_MJD[_leap_idx] && _leap_idx > 0)
        {
            _leap_idx--;
        }

        return LEAP_SECONDS_TAI[_leap_idx];
    }

    inline double to_julian_date() const
    {
        return static_cast<double>(this->_mjdn) + (this->_day_sec*SEC_2_DAY_FRACTION + MJD_JULIAN_DATE);
    }

    static double get_besselian_years(double jd)
    {
        return 1900.0 + (jd - 2415020.31352) / 365.242198781;
    }

};