#pragma once

#include <stdint.h>


template<typename T, uint_fast16_t N>
class CircularArray
{
    static constexpr N_FINAL = N - 1;
    T _data[N];
    T* _end = _data + N;
    T* _head = _data;
    uint_fast16_t _head_idx = 0;

public:

    class iterator
    {
        T* _ptr;
    public:

        T& operator*()
        {
            return *_ptr;
        }
    };

    CircularArray(){}
    ~CircularArray(){}

    inline T& head()
    {
        return *_head;
    }

    inline T* head_ptr()
    {
        return _head;
    }

    inline T& operator[](uint_fast16_t idx)
    {
        auto ptr = idx > _head_idx ? _end - (idx - _head_idx) : _head - idx;
        return *ptr;
    }

    inline void push_empty()
    {
        _head_idx == N_FINAL;

    }

    inline void push(const T& val)
    {

    }

    inline iterator begin()
    {
        return iterator(_head);
    }

};


