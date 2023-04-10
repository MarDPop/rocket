#pragma once

class Gravity
{
public:

    Gravity();
    virtual ~Gravity();

    virtual double get_acceleration(double altitude);
};

class GroundReferenceGravity : public virtual Gravity
{
    const double g0;

    const double R0;

public:
    GroundReferenceGravity(double _g0 = 9.806, double _R0 = 6371000.0);
    virtual ~GroundReferenceGravity();

    double get_acceleration(double altitude) override;
};
