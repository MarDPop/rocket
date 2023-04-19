#pragma once

class SingleStageRocket;

class SingleStageSimulation;

class Loader
{

public:

    static void loadSimulation(SingleStageSimulation& simulation, const char* fn);

    static void loadRocket(SingleStageRocket& rocket, const char* fn);

};
