#pragma once

namespace physics {

    constexpr double KNSU_burn_constant = 0.0016891; //m/s

    constexpr double KNSU_SR_exponent = 0.32;

    void saint_robert_burn(double a, double P, double n);




}
