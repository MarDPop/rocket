#include "../include/Planet.h"

#define TWOPI 6.28318530717958647692528676655

double Ephemeris::trueAnomalyFromEccentricAnomaly( const double EA, const double eccentricity ){
    double beta = eccentricity/(1 + sqrt(1 - eccentricity*eccentricity));
    double s = sin(EA);
    return EA + 2*atan(beta*s/(1 - beta*sqrt(1 - s*s)));
}

double Ephemeris::trueAnomalyFromMeanAnomaly( const double MA, const double eccentricity ){
    return trueAnomalyFromEccentricAnomaly(eccentricAnomalyFromMeanAnomaly(MA,eccentricity),eccentricity);
}

double Ephemeris::trueAnomalyFromMeanAnomalyApprox( const double MA, const double eccentricity ){
    double e2 = eccentricity*eccentricity;
    return MA + eccentricity*(2 - 0.25*e2)*sin(MA) + e2*(1.2*sin(MA+MA) + eccentricity*1.083333333333333*sin(3*MA));
}

double Ephemeris::meanAnomalyFromTrueAnomalyApprox( const double f, const double eccentricity){
    double e2 = eccentricity*eccentricity;
    double e3 = e2*eccentricity;
    double f2 = f+f;
    return f + eccentricity*(-2*sin(f) + (0.75*eccentricity+0.125*e3)*sin(f2) - 0.333333333333333*e2*sin(f2+f)+ 0.15625*e3*sin(f2+f2));
}

double Ephemeris::eccentricAnomalyFromMeanAnomaly( const double MA, const double eccentricity){
    double EA = MA;
    for (int iter = 0; iter < 10; iter++) {
        double dEA = (EA - eccentricity*sin(EA) - MA)/(1 - eccentricity*cos(EA));
        EA -= dEA;
        if (fabs(dEA) < 1e-10)
            break;
    }
    return EA;
}

double Ephemeris::meanAnomalyFromEccentricAnomaly( const double EA, const double eccentricity) {
    return EA - eccentricity*sin(EA);
}

double Ephemeris::eccentricAnomalyFromTrueAnomaly( const double TA, const double eccentricity ) {
    return atan2(sqrt(1 - eccentricity*eccentricity)*sin(TA), eccentricity + cos(TA));
}

double Ephemeris::meanAnomalyFromTrueAnomaly( const double f, const double eccentricity){
    return meanAnomalyFromEccentricAnomaly(eccentricAnomalyFromTrueAnomaly(f,eccentricity),eccentricity);
}

std::array<double,6> Ephemeris::kepler2cartesian(const std::array<double,7>& oe){

    double tmp = 1.0 - oe[1]*oe[1];
    double st = sin(oe[5]);
    double ct = cos(oe[5]);
    double tmp2 = 1.0 + oe[1]*ct;
    double radius = oe[0]*tmp/tmp2;
    double x = radius*ct;
    double y = radius*st;
    tmp = sqrt( oe[6]*oe[0]*tmp )/(radius*tmp2);
    double v_x = -st*tmp;
    double v_y = (oe[1]+ct)*tmp;

    if (fabs(oe[2]) < 1e-8){
        std::array<double,6> out = {x,y,0,v_x,v_y,0};
        return out;
    }

    double cw = cos(oe[4]);
    double sw = sin(oe[4]);
    double co = cos(oe[3]);
    double so = sin(oe[3]);

    st = sin(oe[2]);
    ct = sqrt(1.0-st*st);
    double Rxx = cw*co - sw*ct*so;
    double Rxy = -(sw*co + cw*ct*so);
    double Ryx = cw*so + sw*ct*co;
    double Ryy = cw*ct*co - sw*so;
    double Rzx = sw*st;
    double Rzy = cw*st;

    std::array<double,6> out;
    out[0] = Rxx*x + Rxy*y;
    out[1] = Ryx*x + Ryy*y;
    out[2] = Rzx*x + Rzy*y;
    out[3] = Rxx*v_x + Rxy*v_y;
    out[4] = Ryx*v_x + Ryy*v_y;
    out[5] = Rzx*v_x + Rzy*v_y;
    return out;
}

std::array<double,3> Ephemeris::kepler2position(const std::array<double,6>& oe){
    double st = sin(oe[5]);
    double ct = cos(oe[5]);
    double radius = oe[0]*(1.0 - oe[1]*oe[1])/(1.0 + oe[1]*ct);
    double x = radius*ct;
    double y = radius*st;

    if (fabs(oe[2]) < 1e-8){
        std::array<double,3> out = {x,y,0};
        return out;
    }

    double cw = cos(oe[4]);
    double sw = sin(oe[4]);
    double co = cos(oe[3]);
    double so = sin(oe[3]);

    st = sin(oe[2]);
    ct = sqrt(1.0-st*st);
    double Rxx = cw*co - sw*ct*so;
    double Rxy = -(sw*co + cw*ct*so);
    double Ryx = cw*so + sw*ct*co;
    double Ryy = cw*ct*co - sw*so;
    double Rzx = sw*st;
    double Rzy = cw*st;

    std::array<double,3> out;
    out[0] = Rxx*x + Rxy*y;
    out[1] = Ryx*x + Ryy*y;
    out[2] = Rzx*x + Rzy*y;
    return out;
}

std::array<double,6> Ephemeris::cartesian2kepler(const std::array<double,7>& state) {
    std::array<double,6> oe;
    std::array<double,3> h = {state[1]*state[5] - state[2]*state[4], state[2]*state[3] - state[0]*state[5], state[0]*state[4] - state[1]*state[3]};

    // std::array<double,2> n = {h[1],-h[0]}; // z is implicit 0
    double v2 = state[3]*state[3] + state[4]*state[4] + state[5]*state[5];
    double r_inv = 1/sqrt(state[0]*state[0] + state[1]*state[1] + state[2]*state[2]);
    double rv = state[0]*state[3] + state[1]*state[4] + state[2]*state[5];
    std::array<double,3> e;
    double tmp1 = v2/state[6] - r_inv;
    double tmp2 = rv/state[6];
    e[0] = state[0]*tmp1 + state[3]*tmp2;
    e[1] = state[1]*tmp1 + state[4]*tmp2;
    e[2] = state[2]*tmp1 + state[5]*tmp2;
    double egy = v2/2 - state[6]*r_inv;

    oe[0] = -state[6]/(2*egy);
    oe[1] = sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
    double inv_e = 1/oe[1];
    double nmag = h[0]*h[0]+h[1]*h[1];
    oe[2] = acos(h[2]/sqrt(nmag + h[2]*h[2]));

    if ( fabs(oe[2]) > 1e-9){
        nmag = 1.0/sqrt(nmag);
        oe[3] = acos(h[1]*nmag);
        if (h[0] > 0){
            oe[3] = TWOPI - oe[3];
        }

        oe[4] = acos((h[1]*e[0] - h[0]*e[1])*nmag*inv_e);
        if (e[2] < 0){
            oe[4] = TWOPI - oe[4];
        }
    }

    oe[5] = acos((e[0]*state[0]+e[1]*state[1]+e[2]*state[2])*r_inv*inv_e);
    if (rv < 0){
        oe[5] = TWOPI - oe[5];
    }
    return oe;
}
