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
        T* const _data;
        T* _ptr;

        iterator(T* ptr, T* data) : _ptr(ptr) , _data(data) {}

    public:

        T& operator*()
        {
            return *_ptr;
        }

        T* get()
        {
            return _ptr;
        }

        bool operator++()
        {
            _ptr = _ptr + ((_ptr == _data)*N - 1);
            return _ptr == _data;
        }

        bool operator==(const iterator& it)
        {
            return it._ptr == this._ptr;
        }

        bool operator!=(const iterator& it)
        {
            return it._ptr != this._ptr;
        }
    };

    CircularArray(){}
    ~CircularArray(){}

    inline T& head()
    {
        return *_head;
    }

    inline T& tail()
    {
        return (_head_idx == N_FINAL) ? *_data : *(_head + 1);
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

    inline const T& operator[](uint_fast16_t idx) const
    {
        auto ptr = idx > _head_idx ? _end - (idx - _head_idx) : _head - idx;
        return *ptr;
    }

    inline void push_empty()
    {
        _head = _head + (1 - ((_head == _end)*N);
        _head_idx = _head - _data;
    }

    inline void push(const T& val)
    {
        push_empty();
        *_head = val;
    }

    inline iterator iter()
    {
        return iterator(_head, _data);
    }


};


