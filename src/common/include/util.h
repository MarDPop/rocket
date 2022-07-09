#pragma once

namespace util {

    unsigned int lowerbound(const double* x, const double& val, unsigned int hi){
        unsigned int lo = 0;
        unsigned int mid = (lo + hi)/2;
        while(lo != mid){
            if(x[mid] > val){
                lo = mid;
            } else {
                hi = mid;
            }
            mid = (lo + hi)/2;
        }
        return mid;
    }

}