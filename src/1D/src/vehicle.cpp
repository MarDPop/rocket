#include "../include/Vehicle.h"

#include <cmath>

Vehicle_1D::Vehicle_1D() {

}

Vehicle_1D::~Vehicle_1D() {

}

void Vehicle_1D::launch() {
    this->heights.clear();

    double dt_half = this->time_step*0.5;

    double gas_const = 287.87*this->ground_temperature;

    double drag_const = 0.5*this->CD0*this->Aref / gas_const;

    double time = 0;
    double time_record = this->recording_interval;

    this->heights.push_back(0);

    double height = 0;
    double speed = 0;
    while(true){

        double pressure = this->ground_pressure*exp(height/-8400.0);

        this->thruster->update(pressure,time + dt_half);

        double drag = drag_const*pressure*fabs(speed)*speed;

        double acceleration = (this->thruster->thrust + drag) / this->mass - 9.806;

        double mass_tmp = this->mass - this->thruster->mass_rate*this->time_step;

        double height_tmp = height + speed*this->time_step;

        double speed_tmp = speed + acceleration*this->time_step;

        pressure = this->ground_pressure*exp(height_tmp/-8400.0);

        this->thruster->update(pressure,time + this->time_step);

        drag = drag_const*pressure*fabs(speed_tmp)*speed_tmp;

        double acceleration_tmp = (this->thruster->thrust + drag) / this->mass - 9.806;

        height += (speed + speed_tmp)*dt_half;

        speed += (acceleration + acceleration_tmp)*dt_half;

        time += this->time_step;

        if(time > time_record) {
            this->heights.push_back(height);
            time_record += this->recording_interval;
        }
    }
}
