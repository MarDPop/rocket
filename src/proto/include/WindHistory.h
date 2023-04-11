#pragma once

#include <vector>
#include <string>

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

class WindHistory {

    /* Wind Table */
    std::vector<double> times;
    std::vector<Vector> speed;

    /* iterator positions */
    double* titer;
    Vector* siter;
    double* tend;

    Vector dvdt;

    bool constant = true;

public:

    Vector wind;

    WindHistory();
    ~WindHistory();

    void reset();

    void set(double time);

    void load(std::string fn);

    void load(std::vector<double> t, std::vector<Vector> s);
};
