#pragma once

#include <array>
#include <stdint.h>

template<unsigned int N>
class Dynamics {

public:

    virtual bool stop(){
        return false;
    }

    virtual void get_state_rate(const std::array<double,N>& x, const double& t, std::array<double,N>& dx) = 0;

};
