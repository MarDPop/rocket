#pragma once

namespace physics {

    constexpr double R_GAS = 8.31446261815324; // J/K mol

    constexpr double KNSU_burn_constant = 0.0016891; //m/s

    constexpr double KNSU_SR_exponent = 0.32;

    void saint_robert_burn(double a, double P, double n);




}
