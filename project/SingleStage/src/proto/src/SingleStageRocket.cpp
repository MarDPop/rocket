#include "../include/SingleStageRocket.h"

#include "../../common/include/util.h"
#include "../../../lib/tinyxml/tinyxml2.h"
#include "../include/Action.h"
#include <fstream>
#include <numeric>
#include <iostream>
#include <exception>

void SingleStageRocket::update_inertia()
{
    const Inertia_Basic& fuel_inertia = this->thruster->get_inertia();

    // Get mass first
    this->inertia.mass = this->inertia_empty.mass + fuel_inertia.mass;

    // Compute center of mass
    this->inertia.CoM = this->inertia_empty.CoM*this->inertia_empty.mass;
    // Fuel inertial can be assumed to be in axial
    this->inertia.CoM.z += fuel_inertia.CoM_axial*fuel_inertia.mass;
    this->inertia.CoM *= (1.0/this->inertia.mass);

    // ASSUMPTION: offset CoM contributes negligibly to inertia

    // Off axial terms are just from the structure
    // no parallel axis theory for now, should be neglible
    this->inertia.Ixy = this->inertia_empty.Ixy;
    this->inertia.Ixz = this->inertia_empty.Ixz;
    this->inertia.Iyz = this->inertia_empty.Iyz;

    // Axial terms can be added directly
    this->inertia.Izz = this->inertia_empty.Izz + fuel_inertia.Izz;

    double CoM_z_fuel = fuel_inertia.CoM_axial - this->inertia.CoM.z;
    double CoM_z_struct = this->inertia_empty.CoM.z - this->inertia.CoM.z;
    double parallel_axis_Z = fuel_inertia.mass*CoM_z_fuel*CoM_z_fuel + this->inertia_empty.mass*CoM_z_struct*CoM_z_struct;

    this->inertia.Ixx = this->inertia_empty.Ixx + fuel_inertia.Ixx + parallel_axis_Z;
    this->inertia.Iyy = this->inertia_empty.Iyy + fuel_inertia.Ixx + parallel_axis_Z;
}

void SingleStageRocket::compute_acceleration(double time)
{
    this->_atmosphere->set(this->state.position.z, time);

    this->gnc.update(time);

    BodyAction allActions;
    allActions.location = this->inertia.CoM;

    allActions += this->aerodynamics->update();

    if(this->thruster->is_active())
    {
        allActions += this->thruster->get_action();
    }

    if(this->parachute->is_deployed())
    {
        allActions += this->parachute->update(time);
    }

    Vector total_force = this->state.CS.transpose_mult(allActions.force);

    this->state.acceleration = total_force * (1.0/this->inertia.mass);
    this->state.acceleration.z -= this->_atmosphere->values.gravity;

    // TODO: explore doing in body frame
    Axis body2inertial = this->state.CS.get_transpose();
    // rotate Inertia to inertial frame
    Axis I_inertial = body2inertial * this->inertia.get_inertia_matrix();
    // rotate moment to inertial frame
    Vector total_moment = body2inertial * allActions.moment;
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
        this->thruster->set_time(time);
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

Inertia_Basic loadBasicInertia(tinyxml2::XMLElement* inertiaElement)
{
    Inertia_Basic inertia;
    auto* el = inertiaElement->FirstChildElement("Mass");
    if(!el)
    {
        throw std::invalid_argument("Mass required.");
    }
    inertia.mass = el->DoubleText();

    inertia.Ixx = 0.0;
    inertia.Izz = 0.0;
    inertia.CoM_axial = 0.0;

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
        inertia.CoM_axial = el->DoubleText();
    }

    return inertia;
}

Thruster* loadThruster(tinyxml2::XMLElement* thrusterElement, Atmosphere* atm)
{
    const char* type = thrusterElement->Attribute("Type");
    const char* fn = thrusterElement->Attribute("File");
    Thruster* thruster = nullptr;
    if(!type)
    {
        if(fn)
        {
            thruster = new Thruster(*atm);
            thruster->load(fn);
        }
        else
        {
            auto* inertiaEl = thrusterElement->FirstChildElement("Inertia");
            double thrust = thrusterElement->FirstChildElement("Thrust")->DoubleText();
            double ISP = thrusterElement->FirstChildElement("ISP")->DoubleText();
            thruster = new Thruster(*atm);
            thruster->set_performance(thrust,ISP);
            thruster->set_fuel_inertia(loadBasicInertia(inertiaEl));
        }
    }
    else
    {
        if(strcmp(type,"PressureThruster") == 0)
        {
            if(fn)
            {
                thruster = new PressureThruster(*atm);
                thruster->load(fn);
            }
            else
            {
                PressureThruster* pthruster = new PressureThruster(*atm);
                auto* inertiaEl = thrusterElement->FirstChildElement("Inertia");
                pthruster->set_fuel_inertia(loadBasicInertia(inertiaEl));
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
            thruster = new ComputedThruster(*atm);
            thruster->load(fn);
        }
    }
    return thruster;
}

std::array<double,9> loadSimpleAerodynamicsCoef(tinyxml2::XMLElement* aeroElement)
{
    std::array<double,9> coef;
    coef[0] = aeroElement->FirstChildElement("CD0")->DoubleText();
    coef[1] = aeroElement->FirstChildElement("CL_alpha")->DoubleText();
    coef[2] = aeroElement->FirstChildElement("CM_alpha")->DoubleText();
    coef[3] = aeroElement->FirstChildElement("CM_alpha_dot")->DoubleText();
    coef[4] = aeroElement->FirstChildElement("InducedLiftK")->DoubleText();
    coef[5] = aeroElement->FirstChildElement("RefArea")->DoubleText();
    coef[6] = aeroElement->FirstChildElement("RefLength")->DoubleText();
    coef[7] = aeroElement->FirstChildElement("StallAngle")->DoubleText();
    coef[8] = aeroElement->FirstChildElement("COP")->DoubleText();
    return coef;
}

std::array<double,6> loadFinCoef(tinyxml2::XMLElement* aeroElement)
{
    std::array<double,6> coef;
    coef[0] = aeroElement->FirstChildElement("dCLdTheta")->DoubleText();
    coef[1] = aeroElement->FirstChildElement("dCDdTheta")->DoubleText();
    coef[2] = aeroElement->FirstChildElement("dCMdTheta")->DoubleText();
    coef[3] = aeroElement->FirstChildElement("AreaRef")->DoubleText();
    coef[4] = aeroElement->FirstChildElement("FinZCenter")->DoubleText();
    coef[5] = aeroElement->FirstChildElement("FinSpanCenter")->DoubleText();
    return coef;
}

Aerodynamics* loadAerodynamics(tinyxml2::XMLElement* aeroElement, SingleStageRocket& rocket)
{
    const char* type = aeroElement->Attribute("Type");
    Aerodynamics* aero;
    if(strcmp(type,"SimpleAerodynamics") == 0)
    {
        AerodynamicsBasicCoefficient* basic_aero = new AerodynamicsBasicCoefficient(rocket);
        auto coef = loadSimpleAerodynamicsCoef(aeroElement);
        basic_aero->set_coef(coef);
        aero = basic_aero;
    }
    else if(strcmp(type,"FinAerodynamics") == 0)
    {
        const char* nFinsStr = aeroElement->Attribute("NumberFins");
        if(!nFinsStr)
        {
            throw std::invalid_argument("need to specify number fins");
        }
        unsigned nFins = std::stoi(nFinsStr);

        AerodynamicsFinCoefficient* fin_aero = new AerodynamicsFinCoefficient(rocket,nFins);

        auto coef = loadSimpleAerodynamicsCoef(aeroElement);
        fin_aero->set_coef(coef);
        auto coef_fins = loadFinCoef(aeroElement);
        fin_aero->set_fin_coef(coef_fins);
        aero = fin_aero;
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

    Inertia_Basic inertia = loadBasicInertia(InertiaElement);
    this->inertia_empty.set_from_basic(inertia);

    auto* ThrusterElement = root->FirstChildElement("Thruster");
    if(!ThrusterElement) { throw std::invalid_argument("No thruster"); }

    this->thruster.reset(loadThruster(ThrusterElement, this->_atmosphere));

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

    this->thruster->set_time(0.0);
    this->update_inertia();

    this->_atmosphere->set(0.0,0.0);
    this->gnc.navigation->sensors->calibrate(0.0,this->_atmosphere->values.gravity,this->_atmosphere->values.temperature,this->_atmosphere->values.pressure);

    this->gnc.navigation->filter->init(this->state,0.0);

}

