#pragma once

struct StageDynamics;

class GNC {

    StageDynamics* stage;

public:

    GNC(StageDynamics* s) : stage(s) {}
    ~GNC(){}

};