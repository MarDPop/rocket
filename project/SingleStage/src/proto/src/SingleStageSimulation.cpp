#include "../include/SingleStageSimulation.h"

#include "../../common/include/util.h"
#include <exception>
#include <iostream>
#include <cmath>

#include "../include/Environment.h"

#include "../../../lib/tinyxml/tinyxml2.h"

void SingleStageSimulation::euler_step()
{
    // Get initial state
    this->_rocket->compute_acceleration(this->_time);

    // only update GNC at beginning of time step
    this->_rocket->gnc.update(this->_time); // TODO: Investigate why this breaks things in dynamics when put at start of step

    // propagate to time + dt
    this->_time += this->_dt;

    // Compute mass changes, no need to recompute at next step
    if(this->_rocket->thruster->is_active())
    {
        this->_rocket->thruster->set_time(this->_time);
        this->_rocket->update_inertia(1.0/this->_dt);
    }

    // linear motion
    this->_rocket->state.position += (this->_rocket->state.velocity*this->_dt);
    this->_rocket->state.velocity += (this->_rocket->state.acceleration*this->_dt);

    // Angular motion
    // Rotations are non linear -> rotate CS
    double inertial_rotation_rate = this->_rocket->state.angular_velocity.norm();
    if(inertial_rotation_rate > 1e-8)
    {
        double angle = inertial_rotation_rate*this->_dt;
        Vector axis = this->_rocket->state.angular_velocity * (1.0 / inertial_rotation_rate);
        this->_rocket->state.CS = Axis( angle, axis )*this->_rocket->state.CS;  // confirmed true since rotation matrix is orthogonal
    }

    this->_rocket->state.angular_velocity += (this->_rocket->state.angular_acceleration*this->_dt);
}

void SingleStageSimulation::huen_step()
{
    // Get initial state
    this->_rocket->compute_acceleration(this->_time);

    // only update GNC at beginning of time step
    this->_rocket->gnc.update(this->_time); // TODO: Investigate why this breaks things in dynamics when put at start of step

    // Save initial state
    KinematicState state0 = this->_rocket->state;

    // propagate to time + dt
    this->_time += this->_dt;

    // Compute mass changes, no need to recompute at next step
    if(this->_rocket->thruster->is_active())
    {
        this->_rocket->thruster->set_time(this->_time);
        this->_rocket->update_inertia(1.0/this->_dt);
    }

    // linear motion
    this->_rocket->state.position += (state0.velocity*this->_dt);
    this->_rocket->state.velocity += (state0.acceleration*this->_dt);

    // Angular motion
    // Rotations are non linear -> rotate CS
    double rotation_rate = state0.angular_velocity.norm();
    if(rotation_rate > 1e-8)
    {
        double angle = rotation_rate*this->_dt;
        Vector axis = state0.angular_velocity * (1.0 / rotation_rate);
        this->_rocket->state.CS = Axis( angle, axis )*state0.CS;  // confirmed true since rotation matrix is orthogonal
    }

    this->_rocket->state.angular_velocity += (state0.angular_acceleration*this->_dt);

    /* Average step */

    // recompute state rate at time + dt
    this->_rocket->compute_acceleration(this->_time);

    double dt_half = this->_dt*0.5;

    this->_rocket->state.position = state0.position + (state0.velocity + this->_rocket->state.velocity)*dt_half;
    this->_rocket->state.velocity = state0.velocity + (state0.acceleration + this->_rocket->state.acceleration)*dt_half;

    Vector added_angular_velocity = this->_rocket->state.angular_velocity + state0.angular_velocity;
    rotation_rate = added_angular_velocity.norm();
    if(rotation_rate > 1e-8)
    {
        double angle = rotation_rate*dt_half;
        Vector axis = added_angular_velocity * (1.0 / rotation_rate);
        this->_rocket->state.CS = Axis( angle, axis )*state0.CS;  // confirmed true since rotation matrix is orthogonal
    }
    else
    {
        this->_rocket->state.CS = state0.CS;
    }

    // Average angular rate
    this->_rocket->state.angular_velocity = state0.angular_velocity + (state0.angular_acceleration + this->_rocket->state.angular_acceleration)*dt_half; // Might need to do this after for stability
}

void SingleStageSimulation::euler_heun_adaptive_step()
{
    // Get initial state
    this->_rocket->compute_acceleration(this->_time);

    // only update GNC at beginning of time step
    this->_rocket->gnc.update(this->_time); // TODO: Investigate why this breaks things in dynamics when put at start of step

    // Save initial state
    KinematicState state0 = this->_rocket->state;
    double time0 = this->_time;
    for(int iter = 0; iter < 10; iter++)
    {
        // linear motion
        this->_rocket->state.position = state0.position + (state0.velocity*this->_dt);
        this->_rocket->state.velocity = state0.position + (state0.acceleration*this->_dt);

        // Angular motion
        // Rotations are non linear -> rotate CS
        double rotation_rate = state0.angular_velocity.norm();
        if(rotation_rate > 1e-8)
        {
            double angle = rotation_rate*this->_dt;
            Vector axis = state0.angular_velocity * (1.0 / rotation_rate);
            this->_rocket->state.CS = Axis( angle, axis )*state0.CS;  // confirmed true since rotation matrix is orthogonal
        }

        this->_rocket->state.angular_velocity = state0.angular_velocity + (state0.angular_acceleration*this->_dt);

        // propagate to time + dt
        this->_time = time0 + this->_dt;

        // Compute mass changes, no need to recompute at next step
        if(this->_rocket->thruster->is_active())
        {
            this->_rocket->thruster->set_time(this->_time);
            this->_rocket->update_inertia(1.0/this->_dt);
        }

        Vector position_euler = this->_rocket->state.position;
        Vector Z_axis_euler = this->_rocket->state.CS.axis.z;

        // recompute state rate at time + dt
        this->_rocket->compute_acceleration(this->_time);

        double dt_half = this->_dt*0.5;

        this->_rocket->state.position = state0.position + (state0.velocity + this->_rocket->state.velocity)*dt_half;
        this->_rocket->state.velocity = state0.velocity + (state0.acceleration + this->_rocket->state.acceleration)*dt_half;

        Vector added_angular_velocity = this->_rocket->state.angular_velocity + state0.angular_velocity;
        rotation_rate = added_angular_velocity.norm();
        if(rotation_rate > 1e-8)
        {
            double angle = rotation_rate*dt_half;
            Vector axis = added_angular_velocity * (1.0 / rotation_rate);
            this->_rocket->state.CS = Axis( angle, axis )*state0.CS;  // confirmed true since rotation matrix is orthogonal
        }
        else
        {
            this->_rocket->state.CS = state0.CS;
        }

        // Average angular rate
        this->_rocket->state.angular_velocity = state0.angular_velocity + (state0.angular_acceleration + this->_rocket->state.angular_acceleration)*dt_half; // Might need to do this after for stability

        Vector position_error = position_euler - this->_rocket->state.position;
        double err_squared = position_error.mag();

        double dt_factor_position = sqrt(2.0*this->_position_error_mag/(err_squared + this->_position_error_mag));

        double angle_error = Z_axis_euler.dot(this->_rocket->state.CS.axis.z);

        double dt_factor_angular = 2.0*(1.0 - this->_angle_error_proj) / (2.0 - angle_error - this->_angle_error_proj) ;

        double dt_factor = dt_factor_position*dt_factor_angular;
        this->_dt *= dt_factor;

        if(this->_dt < this->_min_dt)
        {
            this->_dt = this->_min_dt;
            break;
        }

        if(dt_factor > 1.0)
        {
            if(this->_dt > this->_max_dt)
            {
                this->_dt = this->_max_dt;
            }
            break;
        }
    }
}

void SingleStageSimulation::rk23_step()
{

}

SingleStageSimulation::SingleStageSimulation()
{
    // this->step = &SingleStageSimulation::euler_step;
    this->step = &SingleStageSimulation::euler_heun_adaptive_step;
}

SingleStageSimulation::~SingleStageSimulation() {}

void SingleStageSimulation::set_timestep_constraints(double min_dt, double max_dt)
{
    this->_min_dt = min_dt;
    this->_max_dt = max_dt;
}

void SingleStageSimulation::set_error_tolerance(double position_error, double angle_error)
{
    this->_max_angular_error = angle_error;
    this->_max_position_error = position_error;
    this->_position_error_mag = position_error*position_error;
    this->_angle_error_proj = cos(angle_error);
}

void SingleStageSimulation::run(std::string fn, const bool debug)
{
    FILE* output = fopen(fn.c_str(),"w");

    if(!output)
    {
        throw std::invalid_argument("could not open file.");
    }

    fprintf(output,"% 12.9f % 12.9f % 9.5f\n", _launch.latitude, _launch.longitude, _launch.altitude);

    std::ofstream debug_output;
    if(debug)
    {
        std::cout << "Outputting Debug File" << std::endl;
        debug_output.open("debug.txt");
    }

    std::cout << "Running Simulation." << std::endl;

    const std::string CS_FORMAT = "% 14.12f % 14.12f % 14.12f % 14.12f % 14.12f % 14.12f % 14.12f % 14.12f % 14.12f ";
    const std::string POS_FORMAT = "% .6e % .6e % .6e ";
    const std::string OUTPUT_FORMAT = "%8.3f " + POS_FORMAT + CS_FORMAT + "% .6e\n";

    this->_rocket->init(this->_launch.pitch_angle, this->_launch.heading);

    this->_dt = 1.0/512.0;
    this->_time = 0;
    double time_record = 0;

    while(this->_time < 1000.0)
    {
        (this->*step)();

        if(this->_time > time_record)
        {
            if(std::isnan(this->_rocket->state.position.z) || this->_rocket->state.position.z < -0.5) {
                break;
            }

            this->_rocket->state.CS.gram_schmidt_orthogonalize();

            const double* pos = this->_rocket->state.position.data;
            const double* q = this->_rocket->state.CS.data;
            fprintf(output,OUTPUT_FORMAT.c_str(),
                    this->_time, pos[0], pos[1], pos[2], q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], this->_rocket->inertia.mass);

            time_record += this->_record.t_interval;

            if(debug)
            {
                debug_output << this->_time << " ";
                const auto& filtered_state = this->_rocket->gnc.navigation.filter->get_computed_state();
                char buf[150];
                sprintf(buf,POS_FORMAT.c_str(), filtered_state.position[0],filtered_state.position[1],filtered_state.position[2]);
                debug_output << buf;
                sprintf(buf,POS_FORMAT.c_str(), filtered_state.velocity[0],filtered_state.velocity[1],filtered_state.velocity[2]);
                debug_output << buf;
                auto& CS = filtered_state.CS;
                // auto& CS = this->rocket->state.CS;
                sprintf(buf,CS_FORMAT.c_str(),CS.data[0],CS.data[1],CS.data[2],CS.data[3],CS.data[4],CS.data[5],CS.data[6],CS.data[7],CS.data[8]);
                debug_output << buf << std::endl;
            }
        }

        std::cout << "\r" << this->_time << std::flush;
    }

    fclose(output);
}
