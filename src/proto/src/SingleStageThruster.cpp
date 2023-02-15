#include "../include/SingleStageThruster.h"

#include <fstream>
#include <cmath>
#define M_PI 3.14159265358979323846

#include "../../common/include/util.h"
#include "../../common/include/fluid.h"

#include <iostream>

SingleStageThruster::SingleStageThruster() : thrust(0), mass_rate(0) {}
SingleStageThruster::SingleStageThruster(double t, double isp) : thrust(t), mass_rate(t/(isp*9.806)) {}
SingleStageThruster::~SingleStageThruster(){}

void SingleStageThruster::load(std::string fn)
{
    std::ifstream myfile(fn);

    if(myfile.is_open())
    {
        std::string line; getline( myfile, line );
        auto row = util::split(line);
        this->thrust = std::stod(row[0]);
        this->mass_rate = this->thrust/(9.806*std::stod(row[1]));
        myfile.close();
    }
}

void PressureThruster::add_thrust_point(double pressure, double thrust, double mass_rate) {

    if(this->pressures.size() > 0 && pressure > this->pressures.back()) {
        auto i = this->pressures.end();
        while(i != this->pressures.begin()) {
            if(pressure < *(i-1)) {
                break;
            }
            i--;
        }

        int idx = i - this->pressures.begin();

        this->pressures.insert(i,pressure);
        this->thrusts.insert(this->thrusts.begin()+idx,thrust);
        this->mass_rates.insert(this->mass_rates.begin()+idx,mass_rate);
    } else {
        this->pressures.push_back(pressure);
        this->thrusts.push_back(thrust);
        this->mass_rates.push_back(mass_rate);

        this->thrust = thrust;
        this->mass_rate = mass_rate;
    }

    this->idx_final = this->pressures.size()-1;
}

void PressureThruster::reset() {
    this->idx = 0;
};

void PressureThruster::set(double pressure, double time)
{
    while(this->idx < this->idx_final && pressure < this->pressures[this->idx + 1])
    {
        this->idx++;

        double dp = 1.0 / (this->pressures[this->idx+1] - this->pressures[this->idx]);
        this->dT = (this->thrusts[this->idx+1] - this->thrusts[this->idx])*dp;
        this->dM = (this->mass_rates[this->idx+1] - this->mass_rates[this->idx])*dp;
    }

    double delta = pressure - this->pressures[this->idx];
    this->thrust = this->thrusts[this->idx] + this->dT*delta;
    this->mass_rate = this->mass_rates[this->idx] + this->dM*delta;
};

void PressureThruster::load(std::string fn)
{
    std::ifstream myfile(fn);
    pressures.clear();
    thrusts.clear();
    mass_rates.clear();
    if(myfile.is_open())
    {
        for( std::string line; getline( myfile, line ); )
        {
            auto row = util::split(line);
            pressures.push_back(std::stod(row[0]));
            thrusts.push_back(std::stod(row[1]));
            mass_rates.push_back(std::stod(row[2]));
        }
        myfile.close();
    }
    else
    {
        throw std::invalid_argument("could not load file for thruster");
    }
}

ComputedThruster::ComputedThruster(){}

void ComputedThruster::load(std::string fn)
{
    std::ifstream myfile(fn);
    if(myfile.is_open())
    {
        std::vector<std::string> lines;
        bool first_line = true;
        bool precomputed = false;
        for( std::string line; getline( myfile, line ); )
        {
            if(first_line)
            {
                precomputed = (line == "PRECOMPUTED");
                first_line = false;
                continue;
            }

            if(line[0] == '#')
            {
                continue;
            }
            lines.push_back(line);
        }
        myfile.close();

        if(precomputed)
        {
            this->load_precomputed(lines);
        }
        else
        {
            this->load_parameter_set(lines);
        }
    }
    else
    {
        throw std::invalid_argument("could not load file for thruster");
    }
}

void ComputedThruster::load_precomputed(const std::vector<std::string>& lines)
{
    _times.clear();
    _values.clear();
    _dvalues.clear();
    _times.reserve(lines.size());
    _values.reserve(lines.size());
    _dvalues.reserve(lines.size());
    for(auto line : lines)
    {
        auto row = util::split(line);
        _times.push_back(std::stod(row[0]));
        _values.emplace_back(std::stod(row[1]),std::stod(row[2]),std::stod(row[3]),std::stod(row[4]),std::stod(row[5]),std::stod(row[6]),std::stod(row[7]));
    }

    _dvalues.resize(_times.size());

    for(unsigned i = 1; i < _times.size(); i++)
    {
        double dt = 1.0/(_times[i] - _times[i-1]);
        for(unsigned j = 0; j < _values[i].v.size(); j++)
        {
            _dvalues[i].v[j] = (_values[i].v[j] - _values[i-1].v[j])*dt;
        }
    }
}

void ComputedThruster::load_parameter_set(const std::vector<std::string>& lines)
{
    generate_args args;
    if(lines.size() < 11)
    {
        throw std::runtime_error("thruster file does not contain enough parameters");
    }
    double* darg = &args.burn_coef;
    for(int i = 0; i < 12; i++)
    {
        auto row = util::split(lines[i]);
        darg[i] = std::stod(row[1]);
    }
    auto row = util::split(lines[12]);
    args.n_segments = std::stoi(row[1]);

    this->generate(args);
}

inline double exit_mach(double gamma, double area_ratio)
{
    double exp = (gamma+1)/(2*(gamma-1));
    double lhs = area_ratio * pow((gamma+1)*0.5, exp);

    double g = (gamma - 1)*0.5;

    double M_lo = 1.0001;
    double M_hi = 8.0;

    double M_guess = (M_lo + M_hi)*0.5;
    for(int i = 0; i < 20; i++)
    {
        double guess = pow(1 + g*M_guess*M_guess, exp ) / M_guess;
        if(guess > lhs)
        {
            M_hi = M_guess;
        } else {
            M_lo = M_guess;
        }
        M_guess = (M_lo + M_hi)*0.5;
    }
    return M_guess;
}

void ComputedThruster::set_nozzle_parameters(double frozen_gamma, double mw, double throat_area, double exit_area, double half_angle)
{
    this->_gamma = frozen_gamma;
    this->_mw = mw;
    this->_area_ratio = exit_area / throat_area;
    this->_exit_area = exit_area;
    this->_pressure_ratio_critical = pow(2/(frozen_gamma+1), frozen_gamma/(frozen_gamma-1));
    this->_ideal_exit_mach = exit_mach(frozen_gamma, this->_area_ratio);
    this->_half_angle = half_angle;
    this->_half_angle_factor = 0.5*(1 + cos(_half_angle));

    double p_ratio_normal = flow::normal_shock_static_pressure_ratio(this->_ideal_exit_mach,frozen_gamma);
    this->_pressure_ratio_ideal = flow::p_static2p_total(this->_ideal_exit_mach,frozen_gamma);
    this->_pressure_ratio_normal_exit = 1.0/p_ratio_normal;
}

void ComputedThruster::generate(const generate_args& args, const double P_AMBIENT)
{
    /* Get Geometry */
    const double throat_area = M_PI*args.throat_radius*args.throat_radius*args.discharge_coef;
    const double exit_area = M_PI*args.exit_radius*args.exit_radius;

    this->set_nozzle_parameters(args.gamma, args.mw, throat_area, exit_area, 0.2);

    // geometric constants
    const double length_sq = args.length*args.length;
    const double outer_radius_sq = args.radius*args.radius;
    const double N1 = args.n_segments > 1 ? args.n_segments - 1 : 0.0;
    const double Volume_empty = M_PI*outer_radius_sq*args.length;

    /* get gas constants */
    const double R_frozen = flow::R_GAS / args.mw;
    const double k1 = args.gamma - 1;
    const double k2 = args.gamma + 1;
    const double k3 = 1/k1;
    const double ex = -k1/args.gamma;

    const double c = args.gamma*R_frozen;
    const double cp = c/k1;
    const double cv = cp/args.gamma;

    /* Computational Constants */
    const double exit_mach_supersonic = Nozzle::isentropic_exit_mach(this->_area_ratio,args.gamma)*args.nozzle_efficiency;
    const double P_crit = P_AMBIENT/this->_pressure_ratio_critical;
    constexpr double BAR2PASCALS = 100000.0;
    constexpr double P_NORMALIZE = 1.0/BAR2PASCALS;
    const double m_constant = pow(2/k2,0.5*k2/k1)*throat_area;
    const double T_exit_const = 1.0/(1 + k1*0.5*exit_mach_supersonic*exit_mach_supersonic);
    const double dE_dM = args.fuel_heating*args.combustion_efficiency;

    /* Initial Conditions */
    // volume of control volume (m3)
    double V = M_PI*args.bore_radius*args.bore_radius*args.length;
    // pressure of control volume (Pa)
    double P = P_AMBIENT;
    // Temperature control volume (K)
    double T = 300;
    // density control volume (kg/m3)
    double rho = P/(R_frozen*T);
    // mass of control volume (kg)
    double M = V*rho;
    // internal energy of control volume (J)
    double E = M*cv*T;

    /* non control volume variables */
    // Radius of core (m)
    double core_radius = args.bore_radius;
    // Area of burning inside chamber (m2)
    double A_burn = args.bore_radius*M_PI*2*args.length;
    // Mass fuel remaining (kg)
    double M_fuel = (Volume_empty - V)*args.fuel_density;

    /* Mass moment computations */
    double r2 = args.bore_radius*args.bore_radius + outer_radius_sq;
    Ixx = 0.5*M_fuel*r2;
    Izz = M_fuel*(0.25*r2 + 0.0833333333333333333333*length_sq);

    /* Segments  Corrections */
    double core_length = args.length;
    if(args.n_segments > 1)
    {
        double cross_A = N1*M_PI*(outer_radius_sq - args.bore_radius*args.bore_radius);
        double cutVolume = cross_A*args.segment_gap;
        V += cutVolume;
        A_burn += 2*cross_A;
        core_length -= 2*N1*args.segment_gap;

        M_fuel -= cutVolume*args.fuel_density;
    }

    /* Prepare simulation */
    // reset output
    _times.clear();
    _values.clear();
    _dvalues.clear();
    _times.push_back(0.0);
    _values.emplace_back(P,T,0.0,0.0,M_fuel + M,Ixx,Izz);

    // recording and time variables
    double dt = 1e-6; // use variable dt to reduce points
    const int recording_count = 1000;
    double recording_dt = recording_count*dt;
    int cnt = 0;

    /* Compute Main Burn */
    while(M_fuel > 0)
    {
        // mass rate (kg/s)
        double mdot;
        // exit velocity (m/s)
        double exit_v;
        if(P > P_crit)
        {
            exit_v = sqrt(c*T);
            mdot = rho*m_constant*exit_v;
        }
        else
        {
            double exit_mach_sq = (pow(P_AMBIENT/P,ex) - 1)*2*k3;
            double beta = 1 + k1*0.5*exit_mach_sq;
            exit_v = sqrt(exit_mach_sq*c*T/beta);
            double rho_exit = rho*pow(1.0 + k1*0.5*exit_mach_sq,-k3);
            mdot = rho_exit*exit_area*exit_v;
        }

        double burn_rate = args.burn_coef*pow(P*P_NORMALIZE,args.burn_exp);
        double dr = burn_rate*dt;
        double dV = A_burn*dr;
        double dA = 2*M_PI*core_length*dr - N1*4*M_PI*core_radius*dr; // check
        double dM_in = args.fuel_density*dV;
        double dM_out = mdot*dt;
        double dE_in = dE_dM*dM_in;

        double dE_out = ((E+P*V)/M)*dM_out + dM_out*0.5*exit_v*exit_v; // cp*T = E/M

        E += (dE_in - dE_out);
        V += dV;
        A_burn += dA;
        M += (dM_in - dM_out);
        M_fuel -= dM_in;
        core_radius += dr;
        core_length -= 2*N1*dr;

        rho = M/V;
        T = E/(cv*M);
        P = rho*R_frozen*T;

        if(++cnt == recording_count)
        {
            if(P > P_crit)
            {
                double T_exit = T*T_exit_const;
                exit_v = exit_mach_supersonic*sqrt(c*T_exit);
            }
            r2 = core_radius*core_radius + outer_radius_sq;
            Ixx = M_fuel*(0.25*r2 + 0.0833333333333333333333*length_sq);
            Izz = 0.5*(M_fuel*r2 + M*core_radius*core_radius);
            _times.push_back(_times.size()*recording_dt);
            _values.emplace_back(P,T,exit_v,mdot, M_fuel + M,Ixx,Izz);
            cnt = 0;
        }
    }

    /* Compute Fizzle */

    double P_burnout = _values.back().chamber_pressure;
    double Temp_burnout = _values.back().chamber_temperature;
    double burnout_const = -sqrt(R_frozen*T)*m_constant/V;
    double t_burnout = cnt*dt + _times.back();

    for(int i = 0; i < 100; i++)
    {
        double t = i*recording_dt;
        double frac = exp(burnout_const*t);
        P = P_burnout*frac;
        T = Temp_burnout*frac;

        // mass rate (kg/s)
        double mdot;
        // exit velocity (m/s)
        double exit_v;

        if(P > P_crit)
        {
            mdot = rho*m_constant*sqrt(c*T);
            double T_exit = T*T_exit_const;
            exit_v = exit_mach_supersonic*sqrt(c*T_exit);
        }
        else
        {
            double exit_mach_sq = (pow(P_AMBIENT/P,ex) - 1)*2*k3;
            double beta = 1 + k1*0.5*exit_mach_sq;
            exit_v = sqrt(exit_mach_sq*c*T/beta);
            double rho_exit = rho*pow(1.0 + k1*0.5*exit_mach_sq,-k3);
            mdot = rho_exit*exit_area*exit_v;
        }

        M -= mdot*recording_dt;

        if(M < 0)
        {
            break;
        }

        Ixx = M*(0.25*outer_radius_sq + 0.0833333333333333333333*length_sq);
        Izz = 0.5*M*outer_radius_sq;
        _times.push_back(t_burnout + t);
        _values.emplace_back(P,T,exit_v,mdot,M,Ixx,Izz);
    }

    _dvalues.resize(_times.size());

    const unsigned NV = _values[0].v.size();

    for(unsigned i = 1; i < _times.size(); i++)
    {
        double dt = 1.0/(_times[i] - _times[i-1]);
        for(unsigned j = 0; j < NV; j++)
        {
            _dvalues[i].v[j] = (_values[i].v[j] - _values[i-1].v[j])*dt;
        }
    }

}

ComputedThruster::~ComputedThruster(){}

void ComputedThruster::set(double pressure, double time)
{
    while(time > _times[_tidx])
    {
        _tidx++;
    }

    double delta = time - _times[_tidx];

    const auto& vals = this->_values[_tidx];
    const auto& dvals = this->_dvalues[_tidx];

    double chamber_pressure = vals.chamber_pressure + delta*dvals.chamber_pressure;
    double exit_velocity = vals.ideal_exit_velocity + delta*dvals.ideal_exit_velocity;
    this->mass_rate = vals.mass_rate + delta*dvals.mass_rate;
    this->mass = vals.mass + delta*dvals.mass;

    this->Ixx = vals.Ixx + delta*dvals.Ixx;
    this->Izz = vals.Izz + delta*dvals.Izz;

    double pressure_ratio = pressure / chamber_pressure;

    if(pressure_ratio > 1)
    {
        return;
    }

    if(pressure_ratio < this->_pressure_ratio_normal_exit)
    {
        double pressure_exit = chamber_pressure*this->_pressure_ratio_ideal;
        // if(pressure_ratio > this->_pressure_ratio_ideal)
        this->thrust = (pressure_exit - pressure)*this->_exit_area + this->_half_angle_factor*exit_velocity*mass_rate;
    }
    else
    {
        double chamber_temperature = vals.chamber_temperature + delta*dvals.chamber_temperature;
        double temperature_ratio = pow(pressure_ratio,(1.0 - this->_gamma) / this->_gamma);
        double M_exit = sqrt((temperature_ratio - 1.0)*2.0/(this->_gamma - 1.0));
        double v_exit = M_exit*sqrt(this->_gamma*flow::R_GAS/_mw*chamber_temperature/temperature_ratio);

        this->thrust = v_exit*this->mass_rate;
    }
};

void ComputedThruster::save(std::string fn)
{
    std::ofstream file(fn);

    if(file.is_open())
    {
        for(unsigned i = 0; i < this->_times.size();i++)
        {
            file << this->_times[i];
            for(int j = 0; j < 6; j++)
            {
                file << " " << std::to_string(this->_values[i].v[j]);
            }
            file << "\n";
        }

        file << std::endl;
        file.close();
    }
}
