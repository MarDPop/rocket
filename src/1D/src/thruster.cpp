#include "../include/thruster.h"

#include "../../common/include/fluid.h"
#include "../../common/include/util.h"

#include <fstream>
#include <iostream>
#include <cmath>

Combustor::Combustor(){}

Combustor::~Combustor(){}

Nozzle::Nozzle(){}

Nozzle::~Nozzle(){}

double Nozzle::get_thrust(double ambient_pressure) {
    double M_exit = Nozzle::isentropic_exit_mach(this->area_ratio,this->combustor->gamma);
    double beta = 1.0 + 0.5*(this->combustor->gamma-1)*M_exit*M_exit;
    double T_exit = this->combustor->T_total/beta;
    double P_exit = this->combustor->P_total*pow(beta,this->combustor->gamma/(1.0 - this->combustor->gamma));
    double v_exit = sqrt(this->combustor->gamma*this->combustor->R_gas*T_exit);
    return this->combustor->mass_rate*v_exit*this->nozzle_efficiency + (P_exit - ambient_pressure)*this->area_exit;
}

double Nozzle::isentropic_exit_mach(double area_ratio, double k) {
    double k1 = k - 1;
    double k2 = (k + 1)*0.5;
    double ex = k2/k1;
    double ex1 = ex - 1;
    double k1_ex = ex*k1;

    area_ratio *= pow(k2,ex);

    double M_guess = sqrt(area_ratio) + 0.1;

    k1 *= 0.5;

    for(int iter = 0; iter < 50; iter++) {
        double df = 1.0/M_guess;
        double tmp = 1 + k1*M_guess*M_guess;
        double f = pow(tmp,ex)*df;
        df = (pow(tmp,ex1)*(k1_ex*M_guess) - f)*df;
        tmp = (f - area_ratio)/df;
        M_guess -= tmp;
        if(fabs(tmp) < 1e-6) {
            break;
        }
    }
    return M_guess;
}

double Nozzle::isentropic_exit_mach(double area_ratio, double k, double M_guess) {
    double k1 = k - 1;
    double k2 = (k + 1)*0.5;
    double ex = k2/k1;
    double ex1 = ex - 1;
    double k1_ex = ex*k1;

    area_ratio *= pow(k2,ex);

    k1 *= 0.5;

    for(int iter = 0; iter < 10; iter++) {
        double df = 1.0/M_guess;
        double tmp = 1 + k1*M_guess*M_guess;
        double f = pow(tmp,ex)*df;
        df = (pow(tmp,ex1)*(k1_ex*M_guess) - f)*df;
        tmp = (f - area_ratio)/df;
        M_guess -= tmp;
        if(fabs(tmp) < 1e-6) {
            break;
        }
    }
    return M_guess;
}

SolidThruster::SolidThruster() {}

SolidThruster::~SolidThruster() {}

void SolidThruster::update(double ambient_pressure, double time) {
    unsigned int idx = util::bisection_search(this->times.data(),time,this->times.size());

    this->mass_rate = this->mass_rates[idx];
    this->thrust = this->mass_rate*this->v_exit[idx] + (this->p_exit[idx] - ambient_pressure)*this->area_exit;
}

void SolidThruster::save(std::string fn) {
    std::ofstream myfile(fn);
    if(myfile.is_open()){
        const int n = this->times.size();
        for(int i = 0; i < n; i++) {
            myfile << this->times[i] << "," << this->mass_rates[i] << "," << this->p_exit[i] << "," << this->v_exit[i] << std::endl;
        }
        myfile.close();
    }
}

void SolidThruster::load(std::string fn) {
    std::ifstream myfile(fn);
    this->times.clear();
    this->mass_rates.clear();
    this->p_exit.clear();
    this->v_exit.clear();
    if(myfile.is_open()){
        for( std::string line; getline( myfile, line ); ){
            auto row = util::split(line);
            times.push_back(std::stod(row[0]));
            mass_rates.push_back(std::stod(row[1]));
            p_exit.push_back(std::stod(row[2]));
            v_exit.push_back(std::stod(row[3]));
        }
        myfile.close();
    }
}

SugarThruster::SugarThruster() {}

SugarThruster::~SugarThruster() {}

void SugarThruster::compute(double A_b, double V, double M_fuel, double deltaT) {

    this->times.clear();
    this->mass_rates.clear();
    this->p_exit.clear();
    this->v_exit.clear();

    double P = 1e5;
    double T = 300;
    double R_gas = Constants::GAS_CONSTANT/this->fuel->gas_MW;

    double V_final = M_fuel/this->fuel->density + V;

    double k1 = this->fuel->gas_gamma - 1;
    double k2 = this->fuel->gas_gamma + 1;
    double ex = -k1/this->fuel->gas_gamma;

    double cp = this->fuel->gas_gamma*R_gas/k1;

    //double critical_throat_pressure_ratio = pow(2/k2,this->fuel->gas_gamma/k1);

    double exit_mach_subsonic = Nozzle::isentropic_exit_mach(this->area_ratio,this->fuel->gas_gamma, 0.1);
    double p_ratio = flow::p_static2p_total(exit_mach_subsonic,this->fuel->gas_gamma);

    double exit_mach_supersonic = Nozzle::isentropic_exit_mach(this->area_ratio,this->fuel->gas_gamma);
    double exit_beta = 1.0/(1.0 + k1*0.5*exit_mach_supersonic*exit_mach_supersonic);
    double exit_p_ratio = pow(exit_beta,this->fuel->gas_gamma/k1);

    double P_crit = P/p_ratio;
    double m_constant = pow(2/k2,0.5*k2/k1)*this->area_throat;
    //double e_constant = cp/(1+0.5*k1);

    double rho = P/(R_gas*T);
    double M = rho*V;
    double E = M*cp*T;

    double m_dot = 0;
    double time = 0;
    double t_record = 0;
    double dt = 5e-7;
    double dV, RT, M_in, pe,ve;
    double T_burnout = -1;
    double P_burnout = 0;
    double time_burnout = -1;
    double time_fizzle = -1;
    double burnout_const = 0;
    while(time < 10) {

        double M_out = m_dot*dt;

        if(V < V_final) {
            dV = A_b*this->fuel->burn_rate(P,T)*dt;
            M_in = dV*this->fuel->density;

            double dM = M_in - M_out;
            double dE = M_in*this->fuel->heating_value - M_out*cp*T;

            V += dV;
            M += dM;
            E += dE;

            rho = M/V;
            T = E/(cp*M);

            RT = R_gas*T;

            P = rho*RT;

        } else {

            if(time_burnout < 0) {
                time_burnout = time;
                P_burnout = P;
                T_burnout = T;
                burnout_const = -sqrt(RT)*m_constant/V;
            } else {

                if(time_fizzle < 0) {
                    M -= m_dot*dt;
                    rho = M/V;
                    P = rho*RT;

                } else {
                    P = P_burnout*exp(burnout_const*(time-time_burnout));

                    if(P < P_crit) {
                        time_fizzle = time;
                        rho = P/RT;
                        M = rho*V;
                    }

                    T = P/P_burnout*T_burnout;
                    RT = R_gas*T;
                    rho = P/RT;
                }

            }

            if(P < 1.0001e5) {
                break;
            }
        }

        if(P > P_crit) {
            m_dot = rho*sqrt(this->fuel->gas_gamma*RT)*m_constant;
            double te = exit_beta*T;
            pe = exit_p_ratio*P;
            ve = exit_mach_supersonic*sqrt(this->fuel->gas_gamma*R_gas*te);
        } else {
            double M2 = (pow(1e5/P,ex) - 1)*2/k1;
            pe = 1e5;
            ve = sqrt(M2*this->fuel->gas_gamma*RT);
            double rhoe = rho*pow(1.0 + k1*0.5*M2,-1/k1);
            m_dot = rhoe*ve*this->area_exit;
        }

        if(std::isnan(m_dot)){
            break;
        }

        if(time >= t_record) {
            this->times.push_back(time);
            this->mass_rates.push_back(m_dot);
            this->p_exit.push_back(pe);
            this->v_exit.push_back(ve);
            t_record += deltaT;
        }

        time += dt;
    }



}





