#include "../include/Geodesy.h"
#include "../include/Planet.h"
#include <cmath>

double Ellipsoid::vincentyFormulae(double long1, double lat1, double long2, double lat2) {
    double U1 = atan((1 - Earth.EARTH_FLATTENING) * tan(lat1));
    double U2 = atan((1 - Earth.EARTH_FLATTENING) * tan(lat2));
    double L = long2 - long1;
    double l = L;
    double calpha;
    double st;
    double ct;
    double d;
    double a;
    double b;
    calpha = st = ct = d = a = b = 0;
    for (int i = 0; i < 20; i++) {
        double su2 = sin(U2);
        double su1 = sin(U1);
        double cu2 = cos(U2);
        double cu1 = cos(U1);
        a = cu2 * sin(l);
        b = cu1 * su2 - su1 * cu2 * cos(l);
        st = sqrt(a * a + b * b);
        ct = su2 * su1 + cu1 * cu2 * cos(l);
        a = atan2(st, ct);
        b = cu2 * cu1 * sin(l) / st;
        calpha = 1 - b * b;
        d = ct - 2 * su1 * su2 / calpha;
        double C = Earth.EARTH_FLATTENING / 16 * calpha * (4 + Earth.EARTH_FLATTENING * (4 - 3 * calpha));
        l = L + (1 - C) * Earth.EARTH_FLATTENING * b * (a + C * st * (d + C * ct * (2 * d - 1)));
    }
    double u2 = calpha * (Earth.EARTH_EQUATOR_R * Earth.EARTH_EQUATOR_R / (Earth.EARTH_POLAR_R * Earth.EARTH_POLAR_R) - 1);
    double A = 1 - u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)));
    double B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)));
    L = B * st * (d + 0.25 * B * (ct * (2 * d - 1)) - 0.1666 * B * d * (-3 + 4 * b * b) * (-3 + 4 * d));
    return Earth.EARTH_POLAR_R * A * (a - L);
}

Vector Ellipsoid::geodetic2ecef(const Geodetic& lla) {
    double s = sin(latitude);
    double n = Earth.EARTH_EQUATOR_R / sqrt(1.0 - Earth.e2 * s * s);
    double tmp = (n + h) * sqrt(1 - s*s);
    Vector ecef;
    ecef.data[0] = tmp * cos(longitude);
    ecef.data[1] = tmp * sin(longitude);
    ecef.data[2] = s * (n * (1.0 - Earth.e2) + h);
    return ecef;
}

Geodetic Ellipsoid::ecef2geodetic(const Vector& ecef){
    Geodetic geo;   //Results go here (Lat, Lon, Altitude)
    double zp = fabs( ecef.z() );
    double z2 = zp*zp;
    double w2 = ecef.x()*ecef.x() + ecef.y()*ecef.y();
    double w = sqrt( w2 );
    double r2 = 1.0/(w2 + z2);
    double r_inv = sqrt( r2 );
    geo.longitude = atan2( y, x );       //Lon (final)
    double s2 = z2*r2;
    double c2 = w2*r2;
    double u = Earth.a2*r_inv;
    double v = Earth.a3 - Earth.a4*r_inv;
    double s,c,ss;
    if( c2 > 0.3 ){
        s = ( zp*r_inv )*( 1.0 + c2*( Earth.a1 + u + s2*v )*r_inv );
        geo.latitude = asin( s );      //Lat
        ss = s*s;
        c = sqrt( 1.0 - ss );
    } else{
        c = ( w*r_inv )*( 1.0 - s2*( Earth.a5 - u - c2*v )*r_inv );
        geo.latitude = acos( c );      //Lat
        ss = 1.0 - c*c;
        s = sqrt( ss );
    }
    double g = 1.0 - Earth.e2*ss;
    double rg = Earth.EARTH_EQUATOR_R/sqrt( g );
    double rf = Earth.a6*rg;
    u = w - rg*c;
    v = zp - rf*s;
    double f = c*u + s*v;
    double m = c*v - s*u;
    double p = m/( rf/g + f );
    geo.latitude += p;      //Lat
    geo.altitude = f + m*p*0.5;     //Altitude
    if( ecef.z() < 0 ){
        geo.latitude = -geo.latitude;     //Lat
    }
    return geo;    //Return Lat, Lon, Altitude in that order
}
