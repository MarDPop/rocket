#pragma once

#include "Dynamics.h"

#include <cmath>
#include <array>
#include <vector>
#include <stdexcept>

/**
 * @brief A structure to record the states produces by ODE
 * 
 * @tparam NSTATES 
 */
template<unsigned NSTATES>
struct Fixed_Size_Recording
{
    /**
     * @brief the next time to record
     * 
     */
    double time_record = 0.0;

    /**
     * @brief the interval between recording times
     * 
     */
    double recording_interval = 1.0;

    /**
     * @brief recorded states
     * 
     */
    std::vector<std::array<double, NSTATES>> states;

    /**
     * @brief recorded state rates
     * 
     */
    std::vector<std::array<double, NSTATES>> state_rates;

    /**
     * @brief recorded times for states and state rates, with approximately 'recording_interval' between times
     * 
     */
    std::vector<double> times;
};

/**
 * @brief Ordinary Differential Equations Solver
 * 
 * @tparam NSTATES 
 */
template<unsigned NSTATES>
class Fixed_Size_ODE 
{
protected:
    /**
     * @brief the dynamics which determine how state changes
     * 
     */
    Fixed_Size_Dynamics<NSTATES>& _dynamics;

    /**
     * @brief current state
     * 
     */
    std::array<double, NSTATES> _state;

    /**
     * @brief current time
     * 
     */
    double _time; 

    /**
     * @brief current rate of change of state
     * 
     */
    std::array<double, NSTATES> _state_rate; 

    /**
     * @brief current time step
     * 
     */
    double _time_step;     

    /**
     * @brief recording
     * 
     */
    Fixed_Size_Recording<NSTATES> _recording;

    /**
     * @brief records states and increments time to record
     * 
     */
    inline void _record()
    {
        _recording.states.push_back(_state);
        _recording.state_rates.push_back(_state_rate);
        _recording.times.push_back(_time);
        _recording.time_record += _recording.recording_interval;
    }

    inline void _reverse_to_time(double time)
    {
        double time_step_backwards = time - _time;
        for(unsigned i = 0; i < NSTATES; i++)
        {
            _state[i] += _state_rate[i]*time_step_backwards;
        }
        _time = time;
    }

    /**
     * @brief Propagates state and increments current time
     * 
     * @return true if state is valid
     * @return false if state is invalid and ODE needs to stop
     */
    inline virtual bool _step()
    {
        bool valid = this->_dynamics.set_state(this->_state, this->_time, this->_state_rate);
        for(unsigned i = 0; i < NSTATES; i++) 
        {
            this->_state[i] += this->_state_rate[i]*this->_time_step;
        }
        this->_time += this->_time_step;
        return valid;
    }

public:

    Fixed_Size_ODE(Fixed_Size_Dynamics<NSTATES>& dynamics, double time_step = 1.0, double time = 0.0) : 
                     _dynamics(dynamics), _time(time), _time_step(time_step) {}

    virtual ~Fixed_Size_ODE(){}

    inline void set_time(const double& time) 
    {
        this->_time = time;
    }

    inline double get_time() const
    {
        return this->_time;
    }

    inline void set_timestep(const double& time_step) 
    {
        if(time_step == 0.0)
        {
            throw std::invalid_argument("Time step cannot be zero!");
        }
        this->_time_step = time_step;
    }

    inline double get_timestep() const
    {
        return this->_time_step;
    }

    inline void set_state(const std::array<double, NSTATES>& state)
    {
        this->_state = state;
    }

    inline std::array<double, NSTATES> get_state() const 
    {
        return this->_state;
    }

    inline std::array<double, NSTATES> get_state_rate() const 
    {
        return this->_state_rate;
    }

    inline void set_recording_time(double time_record) 
    {
        this->_recording.time_record = time_record;
    }

    inline void set_recording_interval(double recording_interval)
    {
        this->_recording.recording_interval = recording_interval;
    }

    inline void clear_recording()
    {
        this->_recording.states.clear();
        this->_recording.state_rates.clear();
        this->_recording.times.clear();
    }

    inline const Fixed_Size_Recording<NSTATES>& get_recording()  const
    {
        return this->_recording;
    }

    /**
     * @brief propagates state from current time to time input
     * 
     * @param time time 
     */
    inline void run_to_time(double time)
    {
        // reserve space for recording
        unsigned approximate_number_of_records = static_cast<unsigned>(fabs((time - _time) / _recording.recording_interval));
        // Add a bit extra to allow for first 
        constexpr unsigned RECORDING_BUFFER = 3u;
        // New size
        std::size_t buffered_size = _recording.times.size() + approximate_number_of_records + RECORDING_BUFFER;
        _recording.states.reserve(buffered_size);
        _recording.state_rates.reserve(buffered_size);
        _recording.times.reserve(buffered_size);

        // Add initial condition
        _record();

        // Remember _step and _record both increment their time pointers
        while(this->_step() && _time < time) 
        {
            // Record if after interval
            if(_time >= _recording.time_record) _record();
        }
        // if went past the desired time a little go back a partial step
        if(_time > time)
        {
            _reverse_to_time(time);
            // record final state
            _record();
        }
    }
};

/**
 * @brief Options for variable step ODE to control step size
 * 
 * @tparam NSTATES 
 */
template<unsigned NSTATES>
struct Fixed_Size_ODE_Options_Variable
{
    /**
     * @brief 
     * 
     */
    double max_time_step;

    /**
     * @brief 
     * 
     */
    double min_time_step; 

    /**
     * @brief 
     * 
     */
    std::array<double, NSTATES> absolute_error;

    /**
     * @brief 
     * 
     */
    double relative_error;
};

/**
 * @brief ODE base class for ODE's that have variable time steps
 * 
 * @tparam NSTATES 
 */
template<unsigned NSTATES>
class Fixed_Size_ODE_Variable_Step : public virtual Fixed_Size_ODE<NSTATES> 
{
protected:
    /**
     * @brief Variable step options
     * 
     */
    Fixed_Size_ODE_Options_Variable<NSTATES> _options;

public:

    Fixed_Size_ODE_Variable_Step(Fixed_Size_Dynamics<NSTATES>& dynamics, double time_step = 1.0, double time = 0.0) : 
                    Fixed_Size_ODE<NSTATES>(dynamics, time_step, time) {}

    inline void set_options(const Fixed_Size_ODE_Options_Variable<NSTATES>& options) 
    {
        this->_options = options;
    }
};

template<unsigned NSTATES>
class Fixed_Size_ODE_Huen : public virtual Fixed_Size_ODE<NSTATES> 
{
    inline bool _step() override
    {
        this->_dynamics.set_state(this->_state, this->_time, this->_state_rate);
        std::array<double, NSTATES> initial_state = this->_state;
        std::array<double, NSTATES> initial_state_rate = this->_state_rate;

        for(unsigned i = 0; i < NSTATES; i++) 
        {
            this->_state[i] += this->_state_rate[i]*this->_time_step;
        }
        this->_time += this->_time_step;

        bool valid = this->_dynamics.set_state(this->_state, this->_time, this->_state_rate);

        double half_step = 0.5*this->_time_step;
        for(unsigned i = 0; i < NSTATES; i++) 
        {
            this->_state[i] = initial_state[i] + (this->_state_rate[i] + initial_state_rate[i])*half_step;
        }

        return valid;
    }

};

/**
 * @brief A structure to record the states produces by ODE
 * 
 * @tparam NSTATES 
 */
struct Recording
{
    /**
     * @brief the next time to record
     * 
     */
    double time_record = 0.0;

    /**
     * @brief the interval between recording times
     * 
     */
    double recording_interval = 1.0;

    /**
     * @brief recorded states
     * 
     */
    std::vector<fixed_vector<double>> states;

    /**
     * @brief recorded state rates
     * 
     */
    std::vector<fixed_vector<double>> state_rates;

    /**
     * @brief recorded times for states and state rates, with approximately 'recording_interval' between times
     * 
     */
    std::vector<double> times;
};

/**
 * @brief Ordinary Differential Equations Solver
 * 
 * @tparam NSTATES 
 */
class ODE 
{
protected:

    /**
     * @brief the dynamics which determine how state changes
     * 
     */
    Dynamics& _dynamics;

    /**
     * @brief current state
     * 
     */
    fixed_vector<double> _state;

    /**
     * @brief current time
     * 
     */
    double _time; 

    /**
     * @brief current rate of change of state
     * 
     */
    fixed_vector<double> _state_rate; 

    /**
     * @brief current time step
     * 
     */
    double _time_step;     

    /**
     * @brief recording
     * 
     */
    Recording _recording;

    /**
     * @brief records states and increments time to record
     * 
     */
    inline void _record()
    {
        _recording.states.emplace_back(_state);
        _recording.state_rates.emplace_back(_state_rate);
        _recording.times.push_back(_time);
        _recording.time_record += _recording.recording_interval;
    }

    inline void _reverse_to_time(double time)
    {
        double time_step_backwards = time - _time;
        for(unsigned i = 0; i < NSTATES; i++)
        {
            _state[i] += _state_rate[i]*time_step_backwards;
        }
        _time = time;
    }

    /**
     * @brief Propagates state and increments current time
     * 
     * @return true if state is valid
     * @return false if state is invalid and ODE needs to stop
     */
    inline virtual bool _step()
    {
        bool valid = this->_dynamics.set_state(this->_state, this->_time, this->_state_rate);
        for(unsigned i = 0; i < NSTATES; i++) 
        {
            this->_state[i] += this->_state_rate[i]*this->_time_step;
        }
        this->_time += this->_time_step;
        return valid;
    }

public:

    const unsigned NSTATES;

    ODE(Dynamics& dynamics, 
        double time_step = 1.0, 
        double time = 0.0) : 
                            _dynamics(dynamics), 
                            _state(dynamics.NSTATES),
                            _time(time), 
                            _state_rate(dynamics.NSTATES),
                            _time_step(time_step),  
                            NSTATES(dynamics.NSTATES) 
                            {}

    virtual ~ODE(){}

    inline void set_time(const double& time) 
    {
        this->_time = time;
    }

    inline double get_time() const
    {
        return this->_time;
    }

    inline void set_timestep(const double& time_step) 
    {
        if(time_step == 0.0)
        {
            throw std::invalid_argument("Time step cannot be zero!");
        }
        this->_time_step = time_step;
    }

    inline double get_timestep() const
    {
        return this->_time_step;
    }

    inline void set_state(const std::vector<double>& state)
    {
        if(state.size() < NSTATES)
        {
            throw std::invalid_argument("State vector must be at least as large as number of states in dynamics");
        }
        this->_state.set(state.data());
    }

    inline fixed_vector<double> get_state() const 
    {
        return this->_state;
    }

    inline fixed_vector<double> get_state_rate() const 
    {
        return this->_state_rate;
    }

    inline void set_recording_time(double time_record) 
    {
        this->_recording.time_record = time_record;
    }

    inline void set_recording_interval(double recording_interval)
    {
        this->_recording.recording_interval = recording_interval;
    }

    inline void clear_recording()
    {
        this->_recording.states.clear();
        this->_recording.state_rates.clear();
        this->_recording.times.clear();
    }

    inline const Recording& get_recording()  const
    {
        return this->_recording;
    }

    /**
     * @brief propagates state from current time to time input
     * 
     * @param time time 
     */
    inline void run_to_time(double time)
    {
        // reserve space for recording
        unsigned approximate_number_of_records = static_cast<unsigned>(fabs((time - _time) / _recording.recording_interval));
        // Add a bit extra to allow for first 
        constexpr unsigned RECORDING_BUFFER = 3u;
        // New size
        std::size_t buffered_size = _recording.times.size() + approximate_number_of_records + RECORDING_BUFFER;
        _recording.states.reserve(buffered_size);
        _recording.state_rates.reserve(buffered_size);
        _recording.times.reserve(buffered_size);

        // Add initial condition
        _record();

        // Remember _step and _record both increment their time pointers
        while(this->_step() && _time < time) 
        {
            // Record if after interval
            if(_time >= _recording.time_record) _record();
        }
        // if went past the desired time a little go back a partial step
        if(_time > time)
        {
            _reverse_to_time(time);
            // record final state
            _record();
        }
    }
};

/**
 * @brief Options for variable step ODE to control step size
 * 
 * @tparam NSTATES 
 */
struct ODE_Options_Variable
{
    /**
     * @brief 
     * 
     */
    double max_time_step;

    /**
     * @brief 
     * 
     */
    double min_time_step; 

    /**
     * @brief 
     * 
     */
    fixed_vector<double> absolute_error;

    /**
     * @brief 
     * 
     */
    double relative_error;

    ODE_Options_Variable(unsigned NSTATES) : absolute_error(NSTATES) {}
};

/**
 * @brief ODE base class for ODE's that have variable time steps
 * 
 * @tparam NSTATES 
 */
class ODE_Variable_Step : public virtual ODE
{
protected:
    /**
     * @brief Variable step options
     * 
     */
    ODE_Options_Variable _options;

public:

    ODE_Variable_Step(Dynamics& dynamics, double time_step = 1.0, double time = 0.0) : 
                    ODE(dynamics, time_step, time), _options(dynamics.NSTATES) {}

    inline void set_options(const ODE_Options_Variable& options) 
    {
        this->_options = options;
    }
};

class ODE_Huen : public virtual ODE
{
    inline bool _step() override
    {
        this->_dynamics.set_state(this->_state, this->_time, this->_state_rate);
        fixed_vector<double> initial_state = this->_state;
        fixed_vector<double> initial_state_rate = this->_state_rate;

        for(unsigned i = 0; i < NSTATES; i++) 
        {
            this->_state[i] += this->_state_rate[i]*this->_time_step;
        }
        this->_time += this->_time_step;

        bool valid = this->_dynamics.set_state(this->_state, this->_time, this->_state_rate);

        double half_step = 0.5*this->_time_step;
        for(unsigned i = 0; i < NSTATES; i++) 
        {
            this->_state[i] = initial_state[i] + (this->_state_rate[i] + initial_state_rate[i])*half_step;
        }

        return valid;
    }

};


