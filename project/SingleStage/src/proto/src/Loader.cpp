#include "../include/Loader.h"

#include "../../../lib/tinyxml/tinyxml2.h"
#include "../include/SingleStageRocket.h"
#include "../include/SingleStageSimulation.h"

void Loader::loadSimulation(SingleStageSimulation& simulation, const char* fn)
{
    tinyxml2::XMLDocument simDocument;
    auto err = simDocument.LoadFile(fn);
    if(err != tinyxml2::XML_SUCCESS) {
        //Could not load file. Handle appropriately.
        throw std::invalid_argument("Could not load file.");
    }

    auto* root = simDocument.RootElement();
    if(!root) { return; }

    auto* RocketFileElement = root->FirstChildElement("Rocket");
    const char* rocket_fn = RocketFileElement->Attribute("File");

    if(!rocket_fn){ throw std::invalid_argument("No Rocket File."); }

    auto* AtmosphereElement = root->FirstChildElement("Atmosphere");
    if(AtmosphereElement)
    {
        double ground_altitude = AtmosphereElement->FirstChildElement("Altitude")->DoubleText();
        double ground_temperature = AtmosphereElement->FirstChildElement("Temperature")->DoubleText();
        double ground_pressure = AtmosphereElement->FirstChildElement("Pressure")->DoubleText();
        double lapse_rate = AtmosphereElement->FirstChildElement("LapseRate")->DoubleText();
        simulation.atmosphere = std::make_unique<AtmosphereTable>(ground_altitude,ground_pressure,ground_temperature,lapse_rate);
        auto* windEl = AtmosphereElement->FirstChildElement("Wind");
        if(windEl)
        {
            const char* wind_fn = windEl->Attribute("File");
            if(wind_fn)
            {
                simulation.atmosphere->wind.load(wind_fn);
            }
        }
    }
    else
    {
        simulation.atmosphere = std::make_unique<Atmosphere>();
    }

    auto* launchElement = root->FirstChildElement("Launch");
    if(launchElement)
    {
        simulation.launch.latitude = launchElement->FirstChildElement("Latitude")->DoubleText();
        simulation.launch.longitude = launchElement->FirstChildElement("Longitude")->DoubleText();
        simulation.launch.altitude = launchElement->FirstChildElement("Altitude")->DoubleText();
        simulation.launch.pitch_angle = launchElement->FirstChildElement("Pitch")->DoubleText();
        simulation.launch.heading = launchElement->FirstChildElement("Heading")->DoubleText();
    }
    else
    {

    }

    simulation.rocket.reset(new SingleStageRocket(simulation.atmosphere.get()));

    Loader::loadRocket(*simulation.rocket,rocket_fn);
}

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

    if(navigationElement)
    {
        auto* filterElement = navigationElement->FirstChildElement("Filter");
        gnc.navigation.filter = std::make_unique<Filter>();
        if(filterElement)
        {
            const char* type = filterElement->Attribute("Type");
            if(type)
            {
                if(strcmp(type,"SimpleIntegrate") == 0)
                {
                    gnc.navigation.filter = std::make_unique<FilterSimpleIntegrate>();
                } else if(strcmp(type,"Basic") == 0)
                {
                    gnc.navigation.filter = std::make_unique<FilterBasic>();
                }
            }
        }

        gnc.navigation.sensors = std::make_unique<Sensors>();
        auto* sensorElement = navigationElement->FirstChildElement("Sensor");
        if(sensorElement)
        {
            // const char* type = navigationElement->Attribute("Type");
            auto* el = navigationElement->FirstChildElement("Variances");
            if(el)
            {
                double barometer = el->FirstChildElement("Barometer")->DoubleText();
                double thermometer = el->FirstChildElement("Thermometer")->DoubleText();
                double accelerometer = el->FirstChildElement("Accelerometer")->DoubleText();
                double gyro = el->FirstChildElement("Gyro")->DoubleText();
                gnc.navigation.sensors->set_sensor_variances(barometer, thermometer,accelerometer,gyro);
            }
        }
        else
        {
            gnc.navigation.filter = std::make_unique<Filter>();
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

void Loader::loadRocket(SingleStageRocket& rocket, const char* fn)
{
    tinyxml2::XMLDocument simDocument;
    auto err = simDocument.LoadFile(fn);
    if(err != tinyxml2::XML_SUCCESS) { throw std::invalid_argument("Couldn't load file"); }

    auto* root = simDocument.RootElement();
    if(!root) { throw std::invalid_argument("Couldn't find root element"); }

    auto* InertiaElement = root->FirstChildElement("Inertia");
    if(!InertiaElement) { throw std::invalid_argument("No mass properties"); }

    Inertia_Basic inertia = loadBasicInertia(InertiaElement);
    rocket.inertia_empty.set_from_basic(inertia);

    auto* ThrusterElement = root->FirstChildElement("Thruster");
    if(!ThrusterElement) { throw std::invalid_argument("No thruster"); }

    rocket.thruster.reset(loadThruster(ThrusterElement, rocket._atmosphere));

    auto* AerodynamicsElement = root->FirstChildElement("Aerodynamics");
    auto* ParachuteElement = root->FirstChildElement("Parachute");
    auto* GNCElement = root->FirstChildElement("GNC");

    if(!AerodynamicsElement)
    {
        rocket.aerodynamics.reset( new Aerodynamics(rocket));
    }
    else
    {
        rocket.aerodynamics.reset(loadAerodynamics(AerodynamicsElement,rocket));
    }

    if(!ParachuteElement)
    {
        rocket.parachute = std::make_unique<Parachute>(rocket);
    }
    else
    {
        rocket.parachute.reset(loadParachute(ParachuteElement,rocket));
    }

    if(!GNCElement)
    {
        rocket.gnc.guidance = std::make_unique<Guidance>();
        rocket.gnc.control = std::make_unique<Control>();
    }
    else
    {
        loadGNC(rocket.gnc, GNCElement, rocket.parachute.get(), rocket.aerodynamics.get());
    }
}
