#include "../include/Gravity.h"

Gravity::Gravity() {}

Gravity::~Gravity() {}

double Gravity::get_acceleration(double altitude)
{
    return 9.806;
}

GroundReferenceGravity::GroundReferenceGravity(double _g0, double _R0) : g0(_g0), R0(_R0) {}

double GroundReferenceGravity::get_acceleration(double altitude)
{
    double ratio = this->R0/(this->R0 + altitude);
    return g0*ratio*ratio;
}
