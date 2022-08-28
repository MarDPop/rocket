#pragma once

template<class T>
struct ControlInput {

};

class Control {

public:

    Control();
    virtual ~Control();

    void update(double time);

};
