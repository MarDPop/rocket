#include "../include/Servo.h"

#include <cmath>

Servo::~Servo() {}

SimpleServo::SimpleServo() {}

SimpleServo::~SimpleServo() {}

void SimpleServo::update(double time)
{
    double dt = time - this->time_ref;

    this->time_ref = time;

    double angle_to_move = this->commanded_angle - this->angle;

    if(this->torque > this->max_torque && angle_to_move*this->angle > 0)
    {
        return;
    }

    double angle_movement_possible = this->slew_rate*dt;

    if(angle_movement_possible > fabs(angle_to_move))
    {
        angle_movement_possible = angle_to_move;
    }
    else
    {
        angle_movement_possible *= (angle_to_move > 0);
    }

    this->angle += angle_movement_possible;
}
