#pragma once

#include <string>

class IERSData {

    const std::string file_location;

    static IERSData* instance = nullptr;

    IERSData();

public:

    IERSData* get_instance();

};
