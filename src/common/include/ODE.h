#pragma once

#include "Dynamics.h"
#include <array>
#include <vector>
#include <functional>
#include "math.h"
#include <iostream>

#define HUGE_NEG_NUMBER -1e300

template<int N>
class Recording {
    double record_interval;

	std::vector< double > times;

    std::vector< std::array<double,N> > states;

	inline static int bisect(const std::vector<double>& data, const double& val) {
		int lo = 0;
		int hi = data.size()-1;
		int mid = (lo + hi)/2;
		while (lo != mid) {
			if(val > data[mid]) {
				lo = mid;
			} else {
				hi = mid;
			}
			mid = (lo + hi)/2;
		}
		return mid;
	}

public:

	inline void clear() {
		this->times.clear();
		this->states.clear();
	}

	inline void add(const double& time, std::array<double,N> state) {
		this->times.push_back(time);
		this->states.push_back(state);
	}

	inline void set_record_interval(const double& record_interval) {
		this->record_interval = record_interval;
	}

	inline double get_record_interval(){
		return this->record_interval;
	}

	inline int number_entries() const {
		return this->times.size();
	}

	inline std::vector<double> get_times() const {
		return this->times;
	}

	inline std::vector< std::array<double,N> > get_states() const {
		return this->states;
	}

	inline double time_at(int idx) const {
		return this->times[idx];
	}

	inline double final_time() const {
		return this->times.back();
	}

	inline std::array<double,N> state_at(int idx) const {
		return this->states[idx];
	}

    inline std::array<double,N> get(const double& time) const {
        int idx = bisect(this->times,time);
        const std::array<double,N> & lo = this->states[idx];
        const std::array<double,N> & hi = this->states[idx+1];

		std::array<double,N> out;
		double delta = (time - times[idx])/(times[idx+1] - times[idx]);
        for(int i = 0; i < N;i++){
            out[i] = lo[i] + (hi[i] - lo[i])*delta;
        }
        return out;
    }

	inline void save(const std::string& fn){
		FILE * pFile;
		pFile = fopen (fn.c_str(),"w");
		const int n = this->times.size();
		const char* tformat = "%12.6f";
		const char* nformat = " %16.14f";
		for (int i = 0 ; i < n; i++) {
			const std::array<double,N>& state = this->states[i];
			fprintf (pFile, tformat, this->times[i]);
			for(int j = 0; j < N; j++){
				fprintf (pFile, nformat, state[j]);
			}
			fprintf (pFile,'\n');
		}
		fclose (pFile);

	}
};

template<int N>
class ODE_Base {
protected:
    double dt = 1;
    double time = 0;
	std::array<double,N> state;
    Dynamics<N>* dynamics;
    int maxSteps = 10000000;
public:
    Recording<N> recording;

	std::function<bool (const std::array<double,6>&,const double&)> stop = [](const std::array<double,6>&,const double&){return false;};

    virtual void step() = 0;

    inline ODE_Base(){}

    inline virtual ~ODE_Base(){}

    inline void set_dynamics(Dynamics<N>* dynamics){
        this->dynamics = dynamics;
    }

	inline Dynamics<N>* get_dynamics(){
		return this->dynamics;
	}

    virtual void set_timestep(const double& dt) {
        this->dt = dt;
    }

    inline void set_max_steps(const int& maxSteps){
        this->maxSteps = maxSteps;
    }

	inline double get_time() const {
		return this->time;
	}

	inline void set_time(const double& time) {
		this->time = time;
	}

	inline std::array<double,6> get_state() const {
		return this->state;
	}

    void run(const std::array<double,N>& x0, const double& time_final){
        int nSteps = 0;
        double record_time = HUGE_NEG_NUMBER;
		this->state = x0;
        while(nSteps++ < maxSteps && this->time < time_final){

            if(this->time >= record_time){
                this->recording.add(this->time,this->state);
                record_time = this->time + this->recording.get_record_interval();
            }

            this->step();

            if(dynamics->stop() || this->stop(this->state,this->time)){
                break;
            }
        }
		this->recording.add(this->time,this->state);
    }

};

template <int N>
class ODE_Euler : public virtual ODE_Base<N> {
    std::array<double,N> state_rate;
public:
    void step(){
		this->dynamics->get_state_rate(this->state,this->time,this->state_rate);
        for(int i = 0; i < N;i++){
            this->state[i] += this->state_rate[i]*this->dt;
        }
        this->time += this->dt;
    }

};

template <int N>
class ODE_RK4 : public virtual ODE_Base<N> {
	std::array<double,N> k0;
	std::array<double,N> k1;
    std::array<double,N> k2;
    std::array<double,N> k3;
    std::array<double,N> k4;

    double dt2;
	double dt3;
	double dt6;
	double dt8;
public:

	void set_timestep(const double& dt) {
        this->dt = dt;
		this->dt2 = dt/2;
		this->dt3 = dt/3;
		this->dt6 = dt/6;
		this->dt8 = dt/8;
    }

    void step(){
		std::array<double,N> x0 = this->state;
		int i = 0;

		this->dynamics->get_state_rate(this->state,this->time,this->k0);

		while(++i < N){
			this->state[i] = x0[i] + k0[i]*this->dt3;
		}

		this->dynamics->get_state_rate(this->state,this->time + this->dt3, this->k1);
		while(--i >= 0){
			this->state[i] = x0[i] + (this->k0[i] + this->k1[i])*this->dt6;
		}

		this->dynamics->get_state_rate(this->state,this->time + this->dt3, this->k2);
		while(++i < N){
			this->k2[i] *= 3;
			this->state[i] = x0[i] + (this->k0[i] + this->k2[i])*this->dt8;
		}

		this->dynamics->get_state_rate(this->state,this->time + this->dt2, this->k3);
		while(--i >= 0){
			this->k3[i] *= 4;
			this->state[i] = x0[i] + (this->k0[i] - this->k2[i] + this->k3[i])*this->dt2;
		}

		this->dynamics->get_state_rate(this->state,this->time + this->dt,this->k4);
		while(++i < N){
			this->state[i] = x0[i] + (this->k0[i] + this->k3[i] + this->k4[i])*this->dt6;
		}

		this->time += this->dt;
    }

};


template <int N>
class ODE_VariableStep : public virtual ODE_Base<N> {
protected:
    double min_dt;
    double max_dt;

	std::array<double,N> x0;
    std::array<double,N> err;
    std::array<double,N> abs_tol;
public:

    void set_absolute_tol(const std::array<double,N> abs_tol){
        this->abs_tol = abs_tol;
    }

    void set_absolute_tol(const unsigned int& index, const double& val){
        this->abs_tol[index] = val;
    }

    void set_min_timestep(const double& time){
        this->min_dt = time;
    }

    void set_max_timestep(const double& time){
        this->max_dt = time;
    }

};

template <int N>
class ODE_RK23 : public virtual ODE_VariableStep<N> {
    std::array<double,N> k1;
    std::array<double,N> k2;
    std::array<double,N> k3;
    std::array<double,N> k4;
public:

    void step() {
		int i;

		this->x0 = this->state;
        this->dynamics->get_state_rate(this->state,this->time,this->k1);

        int iter = 0;
        while(true) {
            double h = this->dt*0.5;
            for(i = 0; i < N;i++){
                this->state[i] = this->x0[i] + k1[i]*h;
            }
            this->dynamics->calcStateRate(this->state,this->time + h, k2);
            h = this->dt*0.75;
            for(;i != 0;i--){
                this->state[i] = this->x0[i] + k2[i]*h;
            }
            this->dynamics->calcStateRate(this->state,this->time + h, k3);
            for(; i < N;i++){
                this->err[i] = (k1[i]*0.22222222222222 + k2[i]*0.3333333333333333  + k3[i]*0.44444444444444444)*this->dt;
                this->state[i] = this->x0[i] + this->err[i];
            }
            this->dynamics->calcStateRate(this->state,this->time + this->dt,k4);

            double e_max = 0;
            for(; i != 0;i--){
                double e = abs(this->err[i] - this->dt*(k1[i]*0.2916666666666666666 + k2[i]*0.25 + k3[i]*0.33333333333333333 + k4[i]*0.125))/this->abs_tol[i];
                if(e > e_max){
                    e_max = e;
                }
            }

            this->dt *= 0.9/sqrt(e_max); // sqrt(tolerance/e_max)
            if(1 > e_max){
                if(this->dt > this->max_dt) {
                    this->dt = this->max_dt;
                }
                break;
            } else {
                if(this->dt < this->min_dt){
                    this->dt = this->min_dt;
                    break;
                }
                iter++;
                if(iter > 8){
                    break;
                }
				this->k1 = this->k4;
            }

        }
        this->time += this->dt;
    }

};

