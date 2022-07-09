#pragma once

#include <array>
#include <stdint.h>

template<uint_fast16_t N>
class Dynamics {

public:    

    virtual bool stop(){
        return false;
    }   
    
    virtual std::array<double,N> get_state_rate(const std::array<double,N>& x, const double& t) = 0;

};
