#include "../include/SingleStageThruster.h"

#include <fstream>
#include <cmath>
#define M_PI 3.14159265358979323846

#include "../../common/include/util.h"
#include "../../common/include/fluid.h"

#include <iostream>

SingleStageThruster::SingleStageThruster(){}
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
        _values.emplace_back(std::stod(row[1]),std::stod(row[2]),std::stod(row[3]),std::stod(row[4]),std::stod(row[5]),std::stod(row[6]));
    }

    _dvalues.resize(_times.size());

    for(unsigned i = 1; i < _times.size(); i++)
    {
        double dt = 1.0/(_times[i] - _times[i-1]);
        for(unsigned j = 0; j < 6; j++)
        {
            _dvalues[i].v[j] = (_values[i].v[j] - _values[i-1].v[j])*dt;
        }
    }
}

void ComputedThruster::load_parameter_set(const std::vector<std::string>& lines)
{
    std::array<double,11> args;
    if(lines.size() < 11)
    {
        throw std::runtime_error("thruster file does not contain enough parameters");
    }
    for(int i = 0; i < 11; i++)
    {
        auto row = util::split(lines[i]);
        args[i] = std::stod(row[1]);
    }

    this->generate(args[0],args[1],args[2],args[3],args[4],args[5],args[6],args[7],args[8],args[9],args[10]);
}

inline double exit_mach(double gamma, double area_ratio)
{
    double exp = (gamma+1)/(2*(gamma-1));
    double lhs = area_ratio * pow(2/(gamma+1), 1.0/exp);

    double g = (gamma - 1)*0.5;

    double M_lo = 1.0001;
    double M_hi = 8.0;

    double M_guess = (M_lo + M_hi)*0.5;
    for(int i = 0; i < 20; i++)
    {
        double guess = pow(1 + g*M_guess*M_guess, exp ) / M_guess;
        if(guess < lhs)
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
    this->_pressure_ratio_normal_exit = this->_pressure_ratio_ideal * p_ratio_normal;
}

void ComputedThruster::generate(double burn_coef,
                  double burn_exp,
                  double fuel_density,
                  double fuel_heating,
                  double gamma,
                  double mw,
                  double length,
                  double radius,
                  double bore_radius,
                  double throat_radius,
                  double exit_radius)
{
    double throat_area = M_PI*throat_radius*throat_radius;
    double exit_area = M_PI*exit_radius*exit_radius;

    this->set_nozzle_parameters(gamma, mw, throat_area, exit_area, 0.2);

    double R_frozen = flow::R_GAS / mw;
    double k1 = gamma - 1;
    double k2 = gamma + 1;
    double k3 = 1/k1;
    double ex = -k1/gamma;

    double c = gamma*R_frozen;
    double cp = c/k1;

    double exit_mach_supersonic = Nozzle::isentropic_exit_mach(this->_area_ratio,gamma);

    double V = M_PI*bore_radius*bore_radius*length;
    double P = 100000;
    double T = 297;
    double rho = P/(R_frozen*T);
    double M = V*rho; // air kg /m3
    double E = M*cp*T;
    double A_burn = M_PI*bore_radius*2*length;
    double R = bore_radius;

    double L2 = length*length;
    double R2 = radius*radius;

    double P_ambient = P;
    double P_crit = P/this->_pressure_ratio_critical;
    double P_normalize = 1.0/100000.0;
    double m_constant = pow(2/k2,0.5*k2/k1)*throat_area;
    double T_throat_const = 1.0/(1 + k1*0.5);
    double T_exit_const = 1.0/(1 + k1*0.5*exit_mach_supersonic*exit_mach_supersonic);

    double M_fuel = (M_PI*radius*radius*length - V)*fuel_density;

    double dt = 1e-5; // use variable dt to reduce points
    const int recording_count = 100;
    double recording_dt = recording_count*dt;

    double r2 = R*R + R2;
    Ixx = 0.5*M_fuel*r2;
    Izz = M_fuel*(0.25*r2 + 0.0833333333333333333333*L2);

    _times.clear();
    _values.clear();
    _dvalues.clear();
    _times.push_back(0.0);
    _values.emplace_back(P,0.0,0.0,M_fuel + M,Ixx,Izz);

    double mdot = 0;
    double exit_v = 0;
    int cnt = 0;
    while(M_fuel > 0)
    {
        if(P > P_crit)
        {
            exit_v = sqrt(c*T);
            mdot = rho*m_constant*exit_v;
        }
        else
        {
            double exit_mach_sq = (pow(P_ambient/P,ex) - 1)*2*k3;
            double beta = 1 + k1*0.5*exit_mach_sq;
            exit_v = sqrt(exit_mach_sq*c*T/beta);
            double rho_exit = rho*pow(1.0 + k1*0.5*exit_mach_sq,-k3);
            mdot = rho_exit*exit_area*exit_v;
        }

        double burn_rate = burn_coef*pow(P*P_normalize,burn_exp);
        double dr = burn_rate*dt;
        double dV = A_burn*dr;
        double dA = 2*M_PI*length*dr; // check
        double dM_in = fuel_density*dV;
        double dM_out = mdot*dt;
        double dE_in = fuel_heating*dM_in;

        double dE_out = cp*T*dM_out + dM_out*0.5*exit_v*exit_v;

        E += (dE_in - dE_out);
        V += dV;
        A_burn += dA;
        M += (dM_in - dM_out);
        M_fuel -= dM_in;
        R += dr;

        rho = M/V;
        T = E/(cp*M);
        P = rho*R_frozen*T;

        if(++cnt == recording_count)
        {
            if(P > P_crit)
            {
                double T_exit = T*T_exit_const;
                exit_v = exit_mach_supersonic*sqrt(c*T_exit);
            }
            r2 = R*R + R2;
            Ixx = M_fuel*(0.25*r2 + 0.0833333333333333333333*L2);
            Izz = 0.5*(M_fuel*r2 + M*R*R);
            _times.push_back(_times.size()*recording_dt);
            _values.emplace_back(P,exit_v,mdot, M_fuel + M,Ixx,Izz);
            cnt = 0;
        }
    }

    double P_burnout = _values.back().chamber_pressure;
    double burnout_const = -sqrt(R_frozen*T)*m_constant/V;
    double t_burnout = cnt*dt + _times.back();

    for(int i = 0; i < 100; i++)
    {
        double t = i*recording_dt;
        P = P_burnout*exp(burnout_const*t);

        if(P > P_crit)
        {
            mdot = rho*m_constant*sqrt(c*T);
            double T_exit = T*T_exit_const;
            exit_v = exit_mach_supersonic*sqrt(c*T_exit);
        }
        else
        {
            double exit_mach_sq = (pow(P_ambient/P,ex) - 1)*2*k3;
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

        Ixx = M*(0.25*R2 + 0.0833333333333333333333*L2);
        Izz = 0.5*M*R2;
        _times.push_back(t_burnout + t);
        _values.emplace_back(P,exit_v,mdot,M,Ixx,Izz);
    }

    _dvalues.resize(_times.size());

    for(unsigned i = 1; i < _times.size(); i++)
    {
        double dt = 1.0/(_times[i] - _times[i-1]);
        for(unsigned j = 0; j < 6; j++)
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
    mass_rate = vals.mass_rate + delta*dvals.mass_rate;
    mass = vals.mass + delta*dvals.mass;

    Ixx = vals.Ixx + delta*dvals.Ixx;
    Izz = vals.Izz + delta*dvals.Izz;

    double pressure_ratio = pressure / chamber_pressure;

    if(pressure_ratio < this->_pressure_ratio_normal_exit)
    {
        double pressure_exit = chamber_pressure*this->_pressure_ratio_ideal;
        // if(pressure_ratio > this->_pressure_ratio_ideal)
        this->thrust = (pressure_exit - pressure)*this->_exit_area + this->_half_angle_factor*exit_velocity*mass_rate;
    }
    else
    {
        double rhs = pow(pressure_ratio,(this->_gamma - 1) / this->_gamma);
        double M_exit = sqrt((rhs - 1)*2/(this->_gamma - 1));
        double v_exit = M_exit*sqrt(this->_gamma*flow::R_GAS/_mw*1000);

        this->thrust = v_exit*mass_rate;
    }
};

void ComputedThruster::save(std::string fn)
{
    std::ofstream file(fn);

    if(file.is_open())
    {
        for(int i = 0; i < this->_times.size();i++)
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
