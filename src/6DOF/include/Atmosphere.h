#pragma once

#include <array>
#include <cmath>
#include <vector>

#include "../../common/include/Geodesy.h"

struct Air {
    static constexpr double GAMMA = 1.4;
    static constexpr double R_GAS = 8.31446261815324;
    static constexpr double R_DRY_AIR = 287.058;
    static constexpr double DRY_MW = 0.028964399592254;

    union {
        double data[12];
        double pressure;
        double density;
        double speed_of_sound;
        double temperature;
        double dynamic_viscosity;
        double gamma;
        double molecular_weight;
        double specific_gas_constant;
        double absolute_viscosity;
        double cp;
        double cv;
    };

    double wind[3]; // East North Up velocity

};

class Atmosphere {

public:

    Air air;

    double max_height;

    /**
    * returns false if air was computed
    */
    virtual bool get_air(const Geodetic& LLA, double time) = 0;

    Atmosphere(){}
    virtual ~Atmosphere(){}

};

class AtmosphereBasic : public Atmosphere {
    const double scale_height;
    const double sea_pressure;
    const double sea_density;
    const double sea_temp;
    const double R_gas;
public:
    AtmosphereBasic() : scale_height(8.4), sea_pressure(101325), sea_density(1.22), sea_temp(297), R_gas(Air::R_DRY_AIR)  {}

    bool get_air(const Geodetic& LLA, double time) override;
};

class AtmosphereTable : public Atmosphere {

    int idx;

    std::vector< double > altitudes;

    std::vector< std::array< double, 5 > > data;

    std::vector< std::array< double, 5 > > delta;

public:

    AtmosphereTable(){}

    void load(const std::vector<double>& Alt, const std::vector< std::array< double, 5 > >& Data);

    void load(const char* fn);

    void add(double alt, const std::array< double, 5 >& values);

    bool get_air(const Geodetic& LLA, double time) override;

};

class AtmosphereUS1976 : public Atmosphere {


public:

    //pressure(Pa), density (kg/m3), speed_sound (m/s), temp (K), dynamic viscosity (Pa s)
    static const double data[87][5];

    static const double delta[86][5];

    AtmosphereUS1976() {}

    bool get_air(const Geodetic& LLA, double time) override;

};






