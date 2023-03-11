#pragma once

#include <memory>
#include <array>

class SingleStageRocket;

class GNC
{
public:

    const SingleStageRocket& rocket;

    std::unique_ptr<Guidance> guidance;

    std::unique_ptr<Navigation> navigation;

    std::unique_ptr<Control> control;

    GNC(const SingleStageRocket& _rocket);
    virtual ~GNC();

    void update(double time);
};
