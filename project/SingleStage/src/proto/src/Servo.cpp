#include "../include/Servo.h"

#include <cmath>

Servo::~Servo() {}

BoundedServo::BoundedServo(double min_angle, double max_angle) : _min_angle(min_angle),_max_angle(max_angle){}

BoundedServo::~BoundedServo(){}

SlewingServo::SlewingServo() {}

SlewingServo::~SlewingServo() {}

void SlewingServo::update(double time)
{
    double dt = time - this->_time_ref;

    this->_time_ref = time;

    double angle_to_move = this->_commanded_angle - this->_angle;

    if(this->_torque > this->_max_torque && angle_to_move*this->_angle > 0)
    {
        return;
    }

    double angle_movement_possible = this->_slew_rate*dt;

    if(angle_movement_possible > fabs(angle_to_move))
    {
        angle_movement_possible = angle_to_move;
    }
    else
    {
        angle_movement_possible *= (angle_to_move > 0);
    }

    this->_angle += angle_movement_possible;
}
