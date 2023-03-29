#include "../include/Servo.h"

void Servo::update(double torque, double time)
{
    if(torque < this->max_torque)
    {
        double dt = time - this->time_ref;

        double angle_to_move = this->commanded_angle - this->angle;

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

    this->time_ref = time;
}
