#pragma once

#include <vector>
#include <array>
#include <string>

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

    CubicFixedTable( const std::array<double,N>& v, const std::array< std::array< double, COLS >, N >& d) : values(v) , data(d) {
        for(int i = 1; i < N-1; i++) {

        }
    }

}


struct NestedEntry {

    int level;

    std::vector< double > values;

    std::vector< double > delta;

    std::vector< NestedEntry > next;

public:

    NestedEntry(const std::vector< double >& vals) : values(vals) , level(0) {
    }

    NestedEntry(const std::vector< double >& vals, const std::vector< NestedEntry >& n, int l) : values(vals) , next(n), level(l) {

    }

    const std::vector< double > get(const double* key) {

        if(this->level == 0) {
            return this->values;
        }

        if(key[level] < values[0]) {
            return next[0].get(key);
        }

        if(key[level] > values.back()) {
            return next.back().get(key);
        }

        unsigned int lo = util::lowerbound(this->values.data(), key[level], this->values.size()-1);
        unsigned int hi = lo + 1;

        double f_hi = (key[level] - this->values[lo])/(this->values[hi] - this->values[lo]);
        double f_lo = 1 - f_hi;

        const std::vector< double >& l_v = this->next[lo].get(key);
        const std::vector< double >& h_v = this->next[hi].get(key);

        std::vector< double > out(lo.size());
        for( unsigned int i = 0; i < lo.size(); i++) {
            out[i] = f_lo*l_v[i] + f_hi*h_v[i];
        }
        return out;

    }

}

class LinearNestedTable {

    NestedEntry table;

public:

    LinearNestedTable() {}

    LinearNestedTable(std::string file) {}


}



