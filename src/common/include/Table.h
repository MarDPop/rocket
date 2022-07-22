#pragma once

#include <vector>
#include <array>


template < unsigned int N, int COLS >
class LinearFixedTable {
    std::array< double, N> values;
    std::array< std::array< double, COLS >, N > data;
    std::array< std::array< double, COLS >, N > delta;

    inline unsigned int lower_bound(const double& x) {
        unsigned int lo = 0;
        unsigned int hi = N - 1;
        unsigned int mid = (lo + hi) >> 1;
        while(lo != mid) {
            if(x > this->values[mid]) {
                lo = mid;
            } else {
                hi = mid;
            }
            mid = (lo + hi) >> 1;
        }
        return mid;
    }

public:

    LinearFixedTable() {}

    LinearFixedTable( const std::array< double, N >& v, const std::array< std::array< double, COLS >, N >& d) : values(v), data(d) {
        for(unsigned int i = 1; i < N; i++) {
            this->delta[i-1] = (this->data[i] - this->data[i-1])/(this->values[i] - this->values[i-1]);
        }
    }

    inline void get(const unsigned int& x, std::array< double, COLS >& arr) {
        if(x < this->values[0]) {
            arr = this->data[0];
            return;
        }

        if(x > this->values[N-1]) {
            arr = this->data[N-1];
            return;
        }

        unsigned int idx = this->lower_bound(x);
        const double d = x - this->values[idx];
        const std::array<double, COLS>& row = this->data[idx];
        const std::array<double, COLS>& factor = this->delta[idx];
        for(int i = 0; i < COLS; i++) {
            arr[i] = row[i] + d*factor[i];
        }
    }

}

template < unsigned int N, int COLS >
class CubicFixedTable {

    std::array< double, N> values;
    std::array< std::array< double, COLS >, N > data;

    std::array< std::array< std::array< double, 4 > , COLS >, N > coef;

public:

    CubicFixedTable() {}
}

struct NestedEntry {

}

template < unsigned int COLS >
class LinearNestedTable {

    double value;

    int level;

    std::vector< NestedTable* > next;


}



