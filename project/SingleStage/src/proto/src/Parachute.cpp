#include "../include/Parachute.h"

#include "../include/SingleStageRocket.h"

Parachute::Parachute(SingleStageRocket& rocket) : _rocket(rocket)
{
    this->_action.zero();
}

Parachute::Parachute(SingleStageRocket& rocket, double CDA) : _rocket(rocket), _CDA(CDA)
{
    this->_action.zero();
}

Parachute::~Parachute() {}

void Parachute::deploy(double time)
{
    this->_deployed = true;
    this->_time_deployed = time;
}

const BodyAction& Parachute::update(double time)
{
    const auto& air_values = this->_rocket.get_aerodynamics().get_aero_values();
    this->_action.force = air_values.unit_v_air_body * (-this->_CDA*air_values.dynamic_pressure);
    return this->_action;
}

ParachuteTimed::ParachuteTimed(SingleStageRocket& rocket, double CDA, double deployment_time) : Parachute(rocket,CDA),
                                                                                                _deployment_duration(deployment_time) {}

const BodyAction& ParachuteTimed::update(double time)
{
    double dt = time - this->_time_deployed;
    double CDA;
    if(dt > this->_deployment_duration)
    {
        CDA = -this->_CDA;
    }
    else
    {
        CDA = -this->_CDA * dt / this->_deployment_duration;
    }
    const auto& air_values = this->_rocket.get_aerodynamics().get_aero_values();
    this->_action.force = air_values.unit_v_air_body * (CDA*air_values.dynamic_pressure);
    return this->_action;
}

ParachuteModeled::ParachuteModeled(SingleStageRocket& rocket, double deployment_time, double area, double tether_length, double tether_spring_constant, double tether_damper_constant, double chute_mass)
                                            : Parachute(rocket)
{
    this->set_deployment_duration(deployment_time);
    this->set_params(area,tether_length,tether_spring_constant,tether_damper_constant);
    this->_chute_mass = chute_mass;
}

void ParachuteModeled::set_params(double area, double tether_length, double tether_spring_constant, double tether_damper_constant)
{
    static constexpr double CD_HEMISPHERE = 0.9;
    this->_CDA = area*CD_HEMISPHERE*0.5; // 0.5 is premultiplied factor for rho v2
    this->_tether_unstretched_length = tether_length;
    this->_tether_spring_constant = tether_spring_constant;
    this->_tether_damper_constant = tether_damper_constant;
    this->_max_strain = tether_length*0.2;
    this->_volume_deployed = area*sqrt(area)*0.25; // 0.25 is some form factor, smaller for "flatter" parchutes
    this->_chute_position.zero();
    this->_chute_velocity.zero();
}

void ParachuteModeled::deploy(double time)
{
    this->_deployed = true;
    this->_time_deployed = time;
    this->_time_old = time;
    this->_chute_position = this->_rocket.get_state().position;
    this->_chute_velocity = this->_rocket.get_state().velocity;
    this->_old_tether_length = 0.0;
}

const BodyAction& ParachuteModeled::update(double time)
{
    // now calculate dt for kinematics
    double dt = time - this->_time_old;
    this->_time_old = time;

    if(fabs(dt) < 1e-9) {
        return this->_action;
    }

    // calculate dt for deployment
    double dt_deployed = time - this->_time_deployed;
    // Compute deployment
    double CDA, volume;
    if(dt_deployed > this->_deployment_duration)
    {
        CDA = this->_CDA;
        volume = this->_volume_deployed;
    }
    else
    {
        double deployment_factor = dt_deployed / this->_deployment_duration;
        CDA = this->_CDA * deployment_factor;
        volume = this->_volume_deployed * deployment_factor;
    }

    // get air values for aerodynamics
    const auto& air_values = this->_rocket.get_aerodynamics().get_aero_values();
    const auto& rocket_state = this->_rocket.get_state();
    const auto& environment = this->_rocket.get_environment().values;

    // compute total mass, which is air mass and chute mass
    double chute_total_mass = environment.density * volume + this->_chute_mass;

    // compute drag, assume area always normal to air flow
    Vector air_velocity = this->_rocket.get_environment().wind.wind - this->_chute_velocity;
    double airspeed = air_velocity.norm();
    Vector drag = air_velocity * (CDA*environment.density*airspeed);

    // Compute force of tension
    Vector tether = this->_chute_position - rocket_state.position; // remember vector going towards chute, right direction for action
    double tether_length = tether.norm();
    Vector tension_inertial;
    if(tether_length > this->_tether_unstretched_length)
    {
        double strain = std::min(tether_length - this->_tether_unstretched_length, this->_max_strain);
        double strain_rate = (tether_length - this->_old_tether_length)/dt;
        double tension = strain*this->_tether_spring_constant - strain_rate*this->_tether_damper_constant;
        CDA *= (this->_tether_unstretched_length/tether_length); // shrink area due to tension
        tension_inertial = tether * (tension/tether_length);
    }
    else
    {
        tension_inertial.zero();
    }

    this->_old_tether_length = tether_length;

    Vector chute_accel = (drag - tension_inertial) * (1.0/chute_total_mass); // remember tension is equal and opposite for parachute
    chute_accel.z -= environment.gravity;

    this->_chute_position += (this->_chute_velocity*dt);
    this->_chute_velocity += (chute_accel*dt);

    this->_action.force = rocket_state.CS * tension_inertial;
    return this->_action;
}
