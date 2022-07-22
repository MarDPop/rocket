#pragma once

#include <array>
#include <cmath>

struct Air {
    static constexpr double GAMMA = 1.4;
    static constexpr double R_GAS = 8.31446261815324;
    static constexpr double R_DRY_AIR = 287.058;
    static constexpr double DRY_MW = 0.028964399592254;

    union {
        double data[12];
        double density;
        double temperature;
        double pressure;
        double speed_of_sound;
        double kinematic_viscosity;
        double gamma;
        double molecular_weight;
        double specific_gas_constant;
        double absolute_viscosity;
        double cp;
        double cv;
    }

    double wind[3]; // East North Up velocity

};

class Atmosphere {

public:

    /**
    * returns false if air was computed
    */
    virtual bool get_air(Air& air, const std::array<double,3>& LLA) = 0;

};

class AtmosphereBasic : public Atmosphere {
    const double scale_height;
    const double sea_pressure;
    const double sea_density;
    const double sea_temp;
    const double R_gas;
public:
    AtmosphereBasic() : scale_height(8.4), sea_pressure(101325), sea_density(1.22), R_gas(Air::R_DRY_AIR), sea_temp()  {}

    bool get_air(Air& air, const std::array<double,3>& LLA) {
        double factor = exp(-LLA[2]/scale_height);
        air.density = sea_density*factor;
        air.pressure = sea_pressure*factor;

    }
}

class AtmosphereTable : public Atmosphere {

    int idx;

    std::vector< double > altitudes;

    std::vector< std::array< double, 5 > > data;

    std::vector< std::array< double, 5 > > delta;

public:

    AtmosphereTable() :

}

class AtmosphereUS1976 : public Atmosphere {

    static constexpr double data[] = {};

    static constexpr double delta[] = {};

public:

    AtmosphereUS1976() {}

    bool get_air(Air& air, const std::array<double,3>& LLA) {
        if(LLA[2] > 86) {
            return false;
        }

        if(LLA[0] <= 0) {

            return;
        }

        int idx = int(LLA[2]);

        double factor = LLA[2] - idx;


        air.density = this->data[idx]

    }

}






