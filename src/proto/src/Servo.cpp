#include "../include/Servo.h"

void Servo::update(double torque, double time)
{
    if(torque < this->max_torque)
    {
        double dt = time - this->time_ref;

        double angle
    }

    this->time_ref = time;
}
