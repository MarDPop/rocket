#pragma once

#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <exception>
#include <algorithm>

#include "util.h"

class Table
{
    std::vector< double > _delta;

public:

    const std::vector< double > _x;
    const std::vector< double > _v;

    Table( const std::vector< double >& x, const std::vector< double >& v) : _delta(x.size()), _x(x), _v(v)
    {
        for(unsigned i = 1; i < _x.size(); i++)
        {
            this->_delta[i-1] = (this->_v[i] - this->_v[i-1])/(this->_x[i] - this->_x[i-1]);
        }
    }

    inline double get(const double& x) const
    {
        if(x <= _x.front()) { return _v.front(); }

        if(x >= _x.back()) { return _v.back();}

        auto it = std::lower_bound(_x.begin(),_x.end(),x);
        auto idx = std::distance(_x.begin(),it);
        return _v[idx] + (x - *it)*_delta[idx];
    }
};

class IncrementingTable
{
    std::vector< double > _x;
    std::vector< double > _v;
    std::vector< double > _delta;

    double* _x_ptr;
    double* _v_ptr;
    double* _delta_ptr;

public:

    inline void reset()
    {
        if(_x.size() > 1)
        {
            _x_ptr = _x.data() + 1;
            _v_ptr = _v.data() + 1;
            _delta_ptr = _delta.data() + 1;
        }
        else
        {
            _x_ptr = _x.data();
            _v_ptr = _v.data();
            _delta_ptr = _delta.data();
        }
    }

    inline IncrementingTable() { }

    inline void set(const std::vector< double >& x, const std::vector< double >& v)
    {
        this->_x = x;
        this->_v = v;
        this->_delta = std::vector<double>(x.size());
        _delta[0] = 0.0;
        for(unsigned i = 1; i < _x.size(); i++)
        {
            this->_delta[i] = (this->_v[i] - this->_v[i-1])/(this->_x[i] - this->_x[i-1]);
        }
        this->reset();
    }

    inline double get(const double& x)
    {
        if(x <= _x.front()) { return _v.front(); }

        if(x >= _x.back()) { return _v.back();}

        if(x > *_x_ptr)
        {
            _x_ptr++;
            _v_ptr++;
            _delta_ptr++;
            while(x > *_x_ptr)
            {
                _x_ptr++;
                _v_ptr++;
                _delta_ptr++;
            }

        }
        else if (x < *(_x_ptr-1))
        {
            _x_ptr--;
            _v_ptr--;
            _delta_ptr--;
            while(x < *(_x_ptr-1))
            {
                _x_ptr--;
                _v_ptr--;
                _delta_ptr--;
            }
        }
        return *_v_ptr + (x - *_x_ptr)*(*_delta_ptr);
    }

};

template < unsigned int N, int COLS >
class LinearFixedTable {
    std::array< double, N> values;
    std::array< std::array< double, COLS >, N > data;
    std::array< std::array< double, COLS >, N > delta;

public:

    LinearFixedTable() {}

    LinearFixedTable( const std::array< double, N >& v, const std::array< std::array< double, COLS >, N >& d) : values(v), data(d) {
        for(unsigned int i = 1; i < N; i++) {
            this->delta[i-1] = (this->data[i] - this->data[i-1])/(this->values[i] - this->values[i-1]);
        }
    }

    inline void get(const double& x, std::array< double, COLS >& arr) {
        if(x < this->values.front()) {
            arr = this->data.front();
            return;
        }

        if(x > this->values.back()) {
            arr = this->data.back();
            return;
        }

        auto it = std::lower_bound(values.begin(),values.end(),x);
        auto idx = std::distance(values.begin(),it);
        const double d = x - *it;
        const std::array<double, COLS>& row = this->data[idx];
        const std::array<double, COLS>& factor = this->delta[idx];
        for(int i = 0; i < COLS; i++) {
            arr[i] = row[i] + d*factor[i];
        }
    }

};

template < unsigned int N, int COLS >
class CubicFixedTable {

    std::array< double, N> values;
    std::array< std::array< double, COLS >, N > data;

    std::array< std::array< std::array< double, 4 > , COLS >, N > coef;

public:

    CubicFixedTable() {}

    CubicFixedTable( const std::array<double,N>& v, const std::array< std::array< double, COLS >, N >& d) : values(v) , data(d) {
        for(int i = 1; i < N-2; i++) {
            double dx1 = 1/(values[i+1] - values[i-1]);
            double dx2 = 1/(values[i+2] - values[i]);

            double tmp = 1/(values[i+1] - values[i]);
            for(int j = 0; j < COLS; j++) {
                this->coef[i][j][0] = data[i][j]; // d
                this->coef[i][j][1] = (data[i + 1][j] - data[i - 1][j])*dx1; // c
                double dydx2 = (data[i + 2][j] - data[i][j])*dx2;
                this->coef[i][j][2] = 3*tmp*(data[i + 1][j] - this->coef[i][j][0]) - 2*this->coef[i][j][1] - dydx2;
                this->coef[i][j][3] = 0.3333333333333333*tmp*((dydx2 - this->coef[i][j][1])*tmp - 2*this->coef[i][j][2]);
            }
        }
        double dx1 = 1/(values[2] - values[0]);
        double dx2 = 1/(values[N-1] - values[N-3]);
        double tmp1 = 1/(values[1] - values[0]);
        double tmp2 = 1/(values[N-1] - values[N-2]);
        for(int j = 0; j < COLS; j++) {
            this->coef[0][j][0] = data[0][j];
            double dydx1 = (data[2][j] - data[0][j])*dx1;
            this->coef[0][j][2] = (dydx1 + (this->coef[0][j][0] - data[1][j])*dx1)*dx1;
            this->coef[0][j][1] = dydx1 - 2*this->coef[0][j][2]*tmp1;

            this->coef[N-2][j][0] = data[N-2][j];
            this->coef[N-2][j][1] = (data[N-1][j] - data[N-3][j])*dx2;
            this->coef[N-2][j][2] = (data[N-1][j]*tmp2 - this->coef[N-2][j][1])*tmp2;
        }
    }

};

template< unsigned N_KEY, unsigned N_DATA >
class NestedTable {

    const int level;

    std::vector< double > values;

    std::vector< double > delta;

    std::vector< NestedTable > next;

    inline bool is_same(double a, double b) {
        return fabs(a - b) < 1e-7;
    }

    inline NestedTable<N_KEY, N_DATA> process(std::vector< std::array<double, N_KEY + N_DATA > >& data, int level) {
        // ensure sort
        std::sort(data.begin(),data.end(),
                  [level](auto& a, auto& b) {
                    a[level] < b[level];
                  });

        std::vector< double > vals;
        std::vector< NestedTable<N_KEY,N_DATA> > entries;

        if(level == N_KEY - 1) {
            std::array<double,N_DATA> data;
            for(auto& row : data) {
                vals.push_back(row[level]);
                for(int i = 0; i < N_DATA;i++) {
                    data[i] = row[N_KEY + i];
                }
                entries.emplace_back(data);
            }
            return NestedTable<N_KEY, N_DATA>(vals,entries,level);
        }

        std::vector< std::array<double, N_KEY + N_DATA> > next_data;

        unsigned n_rows = data.size();
        unsigned idx = 0;

        while(idx < n_rows) {
            double key_val = data[idx][level];
            vals.push_back(key_val);

            next_data.clear();
            next_data.push_back(data[idx]);
            while(++idx < n_rows && is_same(data[idx][level],key_val)){
                next_data.push_back(data[idx]);
            }

            entries.push_back(process(next_data, level + 1));
        }

        return NestedTable<N_KEY, N_DATA>(vals,entries,level);
    }

    NestedTable(const std::array< double, N_DATA >& vals) : level(N_KEY) {
        this->values.reserve(N_DATA);
        for(auto d : vals){
            this->values.push_back(d);
        }
    }

    NestedTable(const std::vector< double >& vals, const std::vector< NestedTable<N_KEY,N_DATA> >& n, int l) : values(vals) , next(n), level(l) {
        for(unsigned i = 1; i < vals.size(); i++) {
            this->delta.push_back(1.0/(vals[i] - vals[i-1]));
        }
    }

public:

    static NestedTable<N_KEY, N_DATA> create(std::vector< std::array<double, N_KEY + N_DATA > >& data) {
        return process(data, 0);
    }

    static NestedTable<N_KEY, N_DATA> create(std::string fn) {
        std::ifstream file(fn);
        if(!file.is_open()){
            throw std::runtime_error("file does not exist.");
        }

        std::vector< std::array<double, N_KEY + N_DATA > > data;
        std::string line;
        while(getline(file,line)){
            auto row = util::split(line);
            if(row.size() < N_KEY + N_DATA) {
                continue;
            }

            std::array<double, N_KEY + N_DATA > line_data;
            try {
                for(unsigned i = 0; i < N_KEY + N_DATA; i++) {
                    line_data[i] = std::stod(row[i]);
                }
            } catch (...) {
                continue;
            }
            data.push_back(line_data);
        }
        return process(data,0);
    }

    void get(const std::array< double, N_KEY >& key, const std::array< double, N_DATA >& output ) const {

        if(this->level == N_KEY) {
            memcpy(output.data(),this->values.data(),output);
            return;
        }

        auto val = key[level];

        if(val <= values[0]) {
            next[0].get(key,output);
            return;
        }

        if(val >= values.back()) {
            next.back().get(key,output);
            return;
        }

        unsigned lo = 0;
        while(this->values[lo] < val){ // faster for < 10 entries
            lo++;
        }

        double d = (val - this->values[lo])*delta[lo];

        this->next[lo].get(key, output);
        std::array< double, N_DATA > hi;
        this->next[lo + 1].get(key, hi);

        for(unsigned i = 0; i < N_DATA; i++) {
            output[i] += d*hi[i]; // hopefully vectorized
        }
    }

};




