#pragma once

#include <vector>
#include <array>

#include "Cartesian.h"

using namespace Cartesian;

struct Quadratic_fit_precalc {
    std::vector<double> x_sq;
    double inv_num;
    double x_avg;
    double x2_avg;
    double S12;
    double S22;
    double S11;
    double denominator;

    inline Quadratic_fit_precalc(const std::vector<double>& x) {
        const unsigned nX = x.size();

        x_sq.reserve(nX + 6);

        for(unsigned i = 0; i < nX; i++) {
            x_sq[i] = x[i]*x[i];
        }

        inv_num = 1.0 / nX;

        x_avg = x[0];
        x2_avg = x_sq[0];
        S12 = x_sq[0]*x[0];
        S22 = x_sq[0]*x_sq[0];
        for(unsigned i = 1; i < nX; i++) {
            x_avg += x[i];
            x2_avg += x_sq[i];
            S12 += x_sq[i]*x[i];
            S22 += x_sq[i]*x_sq[i];
        }
        double sumX = x_avg;
        double sumX2 = x2_avg;

        x_avg *= inv_num;
        x2_avg *= inv_num;

        S12 -= sumX*x2_avg;
        S22 -= sumX2*x2_avg;

        S11 = sumX2 - sumX*x_avg;
        denominator = 1.0/(S22*S11 - S12*S12);
    }
}

void quadratic_fit_precalced(const std::vector<double>& x, const Quadratic_fit_precalc& precalc,
                   const std::vector<double>& y, std::array<double,3>& coef) {

    const unsigned nX = x.size();
    double y_avg = y[0];
    double Sy1 = y[0]*x[0];
    double Sy2 = y[0]*precalc.x_sq[0];
    for(unsigned i = 1; i < nX; i++) {
        y_avg += y[i];
        Sy1 += y[i]*x[i];
        Sy2 += y[i]*precalc.x_sq[i];
    }

    Sy1 -= y_avg*precalc.x_avg;
    Sy2 -= y_avg*precalc.x2_avg;

    coef[1] = (Sy1*S22 - Sy2*S12)*precalc.denominator;
    coef[2] = (Sy2*S11 - Sy1*S12)*precalc.denominator;

    coef[0] = precalc.inv_num*y_avg - precalc.x_avg*coef[1] - precalc.x2_avg*coef[2];
}

void quadratic_fit_precalced(const std::vector<double>& x, const Quadratic_fit_precalc& precalc,
                   const double* y, unsigned skip, double* coef) {

    const unsigned nX = x.size();
    double y_avg = y[0];
    double Sy1 = y[0]*x[0];
    double Sy2 = y[0]*precalc.x_sq[0];

    double* y_val = y + skip;
    for(unsigned i = 1; i < nX; i++) {
        y_avg += (*y_val);
        Sy1 += (*y_val)*x[i];
        Sy2 += (*y_val)*precalc.x_sq[i];
        y_val += skip;
    }

    Sy1 -= y_avg*precalc.x_avg;
    Sy2 -= y_avg*precalc.x2_avg;

    coef[1] = (Sy1*S22 - Sy2*S12)*precalc.denominator;
    coef[2] = (Sy2*S11 - Sy1*S12)*precalc.denominator;

    coef[0] = precalc.inv_num*y_avg - precalc.x_avg*coef[1] - precalc.x2_avg*coef[2];

}

Axis quadratic_fit(const std::vector<Vector>& time, const std::vector<Vector>& data)
{
    Axis coef;

    Quadratic_fit_precalc precalc(time);

    double* data_ptr = data[0].data;
    double* coef_ptr = Axis.data;
    for(unsigned i = 0; i < 3; i++)
    {
        quadratic_fit_precalced(time,precalc,data_ptr,3,coef_ptr);
        data_ptr++;
        coef_ptr += 3;
    }

    return coef;
}
