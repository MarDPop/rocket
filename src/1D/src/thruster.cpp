#include "../include/thruster.h"

void Thruster::update(double ambient_temperature, double time, double throttle) {

}

Combustor::Combuster(){}

double Nozzle::get_thrust(double ambient_pressure); {
    double M_exit = Nozzle::isentropic_exit_mach(this->Area_ratio,this->combustor->gamma);
    double beta = 1.0 + 0.5*(this->combuster->gamma-1)*M_exit*M_exit;
    double T_exit = this->combustor.T_total/beta;
    double P_exit = this->combustor.P_total*pow(beta,this->combustor->gamma/(1.0 - this->combustor->gamma));
    double v_exit = sqrt(this->combustor->gamma*this->combustor->R_gas*T_exit);
    return this->combuster->mass_rate*v_exit*this->nozzle_efficiency + (P_exit - ambient_pressure)*this->area_exit;
}

double Nozzle::isentropic_exit_mach(double area_ratio, double k) {
    double k1 = k - 1;
    double k2 = (k + 1)*0.5;
    double ex = k2/k1;
    double ex1 = ex - 1;
    double k1_ex = ex*k1;

    area_ratio *= pow(k2,ex);

    double M_guess = 3*(area_ratio - 0.5);

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
