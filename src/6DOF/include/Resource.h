#pragma once

#include <string>

struct Resource
{
    std::string name;
    double mass_per_unit;
    double cost_per_unit;
};

class ResourceQuantity
{
    Resource* resource;
    double quantity;
};
