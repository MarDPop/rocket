#pragma once

#include <array>
#include <algorithm>

/**
 * @brief a class which defines behavior for dynamics which can be implemented by an ODE
 * 
 * @tparam NSTATES 
 */
template<unsigned NSTATES>
struct Fixed_Size_Dynamics 
{
    /**
     * @brief this method describes the rate of change to a state given it's current state and time
     * 
     * @param x current state
     * @param time current time
     * @param dx rate of change of state
     * @return true if state is valid
     * @return false if state is invalid and ODE should stop
     */
    virtual bool set_state(const std::array<double, NSTATES>& x, const double& time, std::array<double,NSTATES>& dx) = 0;
};

template<typename T>
class fixed_vector
{
    T* const _data;
    T* const _end;

public:

    inline fixed_vector(unsigned n) : 
                                    _data(new T[n]), 
                                    _end(_data + n)
                                    {}

    inline ~fixed_vector() 
    {
        delete[] _data;
    }

    inline fixed_vector(const fixed_vector& copy) : 
                                    _data(new T[copy.size()]), 
                                    _end(_data + copy.size())
                                    {
                                        std::copy(copy._data,copy._end,_data);
                                    }

    inline T& operator[](unsigned idx)
    {
        return _data[idx];
    }

    inline T& operator=(const fixed_vector& copy)
    {
        std::copy(copy._data,copy._end,_data);
    }

    inline T* start()
    {
        return _data;
    }

    inline T* end()
    {
        return _end;
    }

    inline unsigned size() const
    {
        return _end - _data;
    }

    inline void set(const T* values)
    {
        T* ptr = _data;
        while(ptr != _end)
        {
            *ptr++ = *values++;
        }
    }
};

/**
 * @brief a class which defines behavior for dynamics which can be implemented by an ODE
 * 
 * 
 */
class Dynamics 
{

public:
    /**
     * @brief number of states
     * 
     */
    const unsigned NSTATES;

    /**
     * @brief this method describes the rate of change to a state given it's current state and time
     * 
     * @param x current state
     * @param time current time
     * @param dx rate of change of state
     * @return true if state is valid
     * @return false if state is invalid and ODE should stop
     */
    virtual bool set_state(const fixed_vector<double>& x, const double& time, fixed_vector<double>& dx) = 0;

    inline Dynamics(unsigned _NSTATES) : NSTATES(_NSTATES) {}
};