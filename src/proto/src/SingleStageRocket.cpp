#include "../include/SingleStageRocket.h"

#include "../../common/include/util.h"
#include "../../../lib/tinyxml/tinyxml2.h"
#include <fstream>
#include <numeric>
#include <iostream>
#include <exception>

void SingleStageRocket::update_inertia()
{
    const auto& fuel_inertia = this->thruster->get_inertia();

    this->inertia.mass = this->inertia_empty.mass + fuel_inertia.mass;
    this->inertia.Izz = this->inertia_empty.Izz + fuel_inertia.Izz;

    this->inertia.COG = (this->inertia_empty.COG*this->inertia_empty.mass + fuel_inertia.COG*fuel_inertia.mass) / this->inertia.mass;

    double cog_arm_fuel = fuel_inertia.COG - this->inertia.COG;
    double cog_arm_structure = this->inertia_empty.COG - this->inertia.COG;
    this->inertia.Ixx = this->inertia_empty.Ixx + fuel_inertia.Ixx;
    this->inertia.Ixx += fuel_inertia.mass*cog_arm_fuel*cog_arm_fuel + this->inertia_empty.mass*cog_arm_structure*cog_arm_structure;
}

void SingleStageRocket::compute_acceleration(double time)
{
    this->_atmosphere->set(this->state.position.z, time);

    this->gnc.update(time);

    this->aerodynamics->update();

    Vector total_force(this->aerodynamics->force);
    Vector total_moment(this->aerodynamics->moment);

    if(this->thruster->is_active())
    {
        this->thruster->set(this->_atmosphere->values.pressure, time);
        total_force += this->state.CS.axis.z * this->thruster->get_thrust();
    }

    if(this->parachute->is_deployed())
    {
        this->parachute->update(time);
        total_force += this->parachute->tether_force;
        total_moment += this->state.CS.axis.z.cross(this->parachute->tether_force) * this->inertia.COG;
    }

    this->state.acceleration = total_force * (1.0/this->inertia.mass);
    this->state.acceleration.z -= this->_atmosphere->values.gravity;

    Axis I_inertial = this->state.CS.get_transpose(); // rotate Inertia to inertial frame
    int i = 0;
    for(; i < 6;i++)
    {
        I_inertial.data[i] *= this->inertia.Ixx;
    }
    for(; i < 9;i++)
    {
        I_inertial.data[i] *= this->inertia.Izz;
    }
    this->state.angular_acceleration = I_inertial.get_inverse() * total_moment;
}

void SingleStageRocket::step(double& time, double dt)
{
    // Get initial state
    this->compute_acceleration(time);

    KinematicState state0 = this->state;

    // propagate to time + dt
    time += dt;

    this->state.position += this->state.velocity*dt;
    this->state.velocity += this->state.acceleration*dt;
    Vector angle_axis = this->state.angular_velocity*dt;
    this->state.angular_velocity += this->state.angular_acceleration*dt;

    // Rotations are non linear -> rotate CS
    double angle = angle_axis.norm();
    if(angle > 1e-6)
    {
        angle_axis *= (1.0/angle);
        this->state.CS = Axis(angle,angle_axis)*state0.CS;
    }

    // Compute mass changes, no need to recompute at next step
    if(this->thruster->is_active())
    {
        this->inertia.mass -= this->thruster->get_mass_rate()*dt;
        this->update_inertia();
    }

    /* Huen step */

    // recompute state rate at time + dt
    this->compute_acceleration(time);

    double dt_half = dt*0.5;

    this->state.position = state0.position + (state0.velocity + this->state.velocity)*dt_half;
    this->state.velocity = state0.velocity + (state0.acceleration + this->state.acceleration)*dt_half;

    angle_axis = (state0.angular_velocity + this->state.angular_velocity)*dt_half;
    this->state.angular_velocity = state0.angular_velocity + (state0.angular_acceleration + this->state.angular_acceleration)*dt_half;

    angle = angle_axis.norm();
    if(angle > 1e-6)
    {
        angle_axis *= (1.0/angle);
        this->state.CS = Axis(angle,angle_axis)*state0.CS;
    }
}
SingleStageRocket::SingleStageRocket(Atmosphere* atmosphere) : gnc(*this), _atmosphere(atmosphere) {}

SingleStageRocket::~SingleStageRocket(){}

Inertia loadInertia(tinyxml2::XMLElement* inertiaElement)
{
    Inertia inertia;
    auto* el = inertiaElement->FirstChildElement("Mass");
    if(!el)
    {
        throw std::invalid_argument("Mass required.");
    }
    inertia.mass = el->DoubleText();

    inertia.Ixx = 0;
    inertia.Izz = 0;
    inertia.COG = 0;

    el = inertiaElement->FirstChildElement("Ixx");
    if(el)
    {
        inertia.Ixx = el->DoubleText();
    }

    el = inertiaElement->FirstChildElement("Izz");
    if(el)
    {
        inertia.Izz = el->DoubleText();
    }

    el = inertiaElement->FirstChildElement("COG");
    if(el)
    {
        inertia.COG = el->DoubleText();
    }

    return inertia;
}

Thruster* loadThruster(tinyxml2::XMLElement* thrusterElement)
{
    const char* type = thrusterElement->Attribute("Type");
    const char* fn = thrusterElement->Attribute("File");
    Thruster* thruster = nullptr;
    if(!type)
    {
        if(fn)
        {
            thruster = new Thruster();
            thruster->load(fn);
        }
        else
        {
            auto* inertiaEl = thrusterElement->FirstChildElement("Inertia");
            double thrust = thrusterElement->FirstChildElement("Thrust")->DoubleText();
            double ISP = thrusterElement->FirstChildElement("ISP")->DoubleText();
            thruster = new Thruster(thrust,ISP);
            thruster->set_fuel_inertia(loadInertia(inertiaEl));
        }
    }
    else
    {
        if(strcmp(type,"PressureThruster") == 0)
        {
            if(fn)
            {
                thruster = new PressureThruster();
                thruster->load(fn);
            }
            else
            {
                PressureThruster* pthruster = new PressureThruster();
                auto* inertiaEl = thrusterElement->FirstChildElement("Inertia");
                pthruster->set_fuel_inertia(loadInertia(inertiaEl));
                auto* pressureTable = thrusterElement->FirstChildElement("Table");
                if(!pressureTable)
                {
                    throw new std::invalid_argument("No table for pressure thruster.");
                }
                auto* row = pressureTable->FirstChildElement();
                while(row)
                {
                    double pressure = std::stod(row->Attribute("Pressure"));
                    double thrust = std::stod(row->Attribute("Thrust"));
                    double ISP = std::stod(row->Attribute("ISP"));
                    pthruster->add_thrust_point(pressure,thrust,thrust/(ISP*9.806));
                    row = row->NextSiblingElement();
                }
                thruster = pthruster;
            }
        }
        else if(strcmp(type,"ComputedThruster") == 0)
        {
            thruster = new ComputedThruster();
            thruster->load(fn);
        }
    }
    return thruster;
}

Aerodynamics* loadSimpleAerodynamics(tinyxml2::XMLElement* aeroElement, SingleStageRocket& rocket)
{
    AerodynamicsBasicCoef* aero = new AerodynamicsBasicCoef(rocket);
    double coef[8];
    coef[0] = aeroElement->FirstChildElement("CD0")->DoubleText();
    coef[1] = aeroElement->FirstChildElement("CL_alpha")->DoubleText();
    coef[2] = aeroElement->FirstChildElement("CM_alpha")->DoubleText();
    coef[3] = aeroElement->FirstChildElement("CM_alpha_dot")->DoubleText();
    coef[4] = aeroElement->FirstChildElement("InducedLiftK")->DoubleText();
    coef[5] = aeroElement->FirstChildElement("RefArea")->DoubleText();
    coef[6] = aeroElement->FirstChildElement("RefLength")->DoubleText();
    coef[7] = aeroElement->FirstChildElement("StallAngle")->DoubleText();
    aero->set_coef(coef);
    return aero;
}

Aerodynamics* loadAerodynamics(tinyxml2::XMLElement* aeroElement, SingleStageRocket& rocket)
{
    const char* type = aeroElement->Attribute("Type");
    Aerodynamics* aero;
    if(strcmp(type,"SimpleAerodynamics") == 0)
    {
        aero = loadSimpleAerodynamics(aeroElement,rocket);
    }
    else
    {
        aero = new Aerodynamics(rocket);
    }
    return aero;
}

Parachute* loadParachute(tinyxml2::XMLElement* chuteElement, SingleStageRocket& rocket)
{
    const char* type = chuteElement->Attribute("Type");
    Parachute* chute;
    double CDA = 0;
    if(!type)
    {
        auto* CDAEl = chuteElement->FirstChildElement("CDA");
        if(CDAEl)
        {
            CDA = CDAEl->DoubleText();
        }
        chute = new Parachute(rocket,CDA);
    }
    else
    {
        chute = new Parachute(rocket,CDA);
    }
    return chute;
}

void loadGNC(GNC& gnc, tinyxml2::XMLElement* gncElement, Parachute* parachute, Aerodynamics* aero)
{
    auto* guidanceElement = gncElement->FirstChildElement("Guidance");
    auto* navigationElement = gncElement->FirstChildElement("Navigation");
    auto* controlElement = gncElement->FirstChildElement("Control");

    gnc.guidance = std::make_unique<Guidance>();
    if(guidanceElement)
    {
        const char* type = guidanceElement->Attribute("Type");
        if(type)
        {
            if(strcmp(type,"VerticalAscent") == 0)
            {
                double P = guidanceElement->FirstChildElement("Proportional")->DoubleText();
                double D = guidanceElement->FirstChildElement("Damping")->DoubleText();
                GuidanceVerticalAscent* guidance = new GuidanceVerticalAscent();
                guidance->setProportionalConstants(P,D);
                guidance->chute = parachute;
                gnc.guidance.reset(guidance);
            }
        }
    }

    gnc.navigation = std::make_unique<Navigation>();
    if(navigationElement)
    {
        auto* filterElement = navigationElement->FirstChildElement("Filter");
        if(filterElement)
        {
            // const char* type = navigationElement->Attribute("Type");
        }
    }

    if(controlElement)
    {
        const char* type = controlElement->Attribute("Type");
        gnc.control = std::make_unique<Control>();
        if(type)
        {
            if(strcmp(type,"FinControl") == 0)
            {
                auto* finAero = dynamic_cast<FinControlAero*>(aero);
                if(!finAero)
                {
                    throw std::invalid_argument("Fin control requires fin aero.");
                }
                double P = controlElement->FirstChildElement("Proportional")->DoubleText();
                double D = controlElement->FirstChildElement("Damping")->DoubleText();
                double F = controlElement->FirstChildElement("FinGain")->DoubleText();
                ControlFinSimple* finControl = new ControlFinSimple(*finAero); // double check this!
                finControl->set_fin_gain(F);
                finControl->set_controller_values(P,D);
                gnc.control.reset(finControl);
            }
        }
    }
}

void SingleStageRocket::load(const char* fn)
{
    tinyxml2::XMLDocument simDocument;
    auto err = simDocument.LoadFile(fn);
    if(err != tinyxml2::XML_SUCCESS) { throw std::invalid_argument("Couldn't load file"); }

    auto* root = simDocument.RootElement();
    if(!root) { throw std::invalid_argument("Couldn't find root element"); }

    auto* InertiaElement = root->FirstChildElement("Inertia");
    if(!InertiaElement) { throw std::invalid_argument("No mass properties"); }

    this->inertia = loadInertia(InertiaElement);

    auto* ThrusterElement = root->FirstChildElement("Thruster");
    if(!ThrusterElement) { throw std::invalid_argument("No thruster"); }

    this->thruster.reset(loadThruster(ThrusterElement));

    auto* AerodynamicsElement = root->FirstChildElement("Aerodynamics");
    auto* ParachuteElement = root->FirstChildElement("Parachute");
    auto* GNCElement = root->FirstChildElement("GNC");

    if(!AerodynamicsElement)
    {
        this->aerodynamics.reset( new Aerodynamics(*this));
    }
    else
    {
        this->aerodynamics.reset(loadAerodynamics(AerodynamicsElement,*this));
    }

    if(!ParachuteElement)
    {
        this->parachute = std::make_unique<Parachute>(*this);
    }
    else
    {
        this->parachute.reset(loadParachute(ParachuteElement,*this));
    }

    if(!GNCElement)
    {
        this->gnc.guidance = std::make_unique<Guidance>();
        this->gnc.control = std::make_unique<Control>();
        this->gnc.navigation = std::make_unique<Navigation>();
    }
    else
    {
        loadGNC(this->gnc, GNCElement, this->parachute.get(), this->aerodynamics.get());
    }


}

void SingleStageRocket::init(double launch_angle, double launch_heading)
{
    this->state.position.zero();
    this->state.velocity.zero();
    this->state.acceleration.zero();
    this->state.angular_velocity.zero();
    this->state.angular_acceleration.zero();

    double cphi = cos(launch_heading);
    double sphi = sin(launch_heading);
    double stheta = sin(1.5707963267948966192 - launch_angle);
    double ctheta = sqrt(1 - stheta*stheta);

    // z rotation by launch_heading
    // y rotation by launch_angle
    // y axis points north at zero heading
    this->state.CS.axis.y.x = sphi;
    this->state.CS.axis.y.y = cphi;
    this->state.CS.axis.y.z = 0.0;
    this->state.CS.axis.z.x = ctheta*cphi;
    this->state.CS.axis.z.y = ctheta*-sphi;
    this->state.CS.axis.z.z = stheta;
    Vector::cross(this->state.CS.axis.y,this->state.CS.axis.z,this->state.CS.axis.x);
}

