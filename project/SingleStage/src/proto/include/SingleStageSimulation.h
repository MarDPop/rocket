#pragma once

#include "SingleStageRocket.h"
#include "Environment.h"
#include <string>
#include <vector>
#include <array>
#include <memory>

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

/**
* Internal struct definition to record states
*/
struct Recording
{
    /**
    * Time interval between recordings (sec)
    */
    double t_interval = 1.0/8.0;

    /**
    * Position (m)
    */
    std::vector<Vector> position;

    /**
    * Full CS orientation
    */
    std::vector<Axis> orientation;

    /**
    * Mass (kg)
    */
    std::vector<double> mass;

    /**
    * Any other test value we want
    */
    std::vector<double> test_value;
};

struct Launch_Parameters
{
    double latitude = 39.94426809919236;
    double longitude = -104.94474985717818;
    double altitude = 1606.0;
    double heading = 0.0;
    double pitch_angle = 0.0;
};

class SingleStageSimulation
{
friend class Loader;

    Launch_Parameters _launch;

    std::unique_ptr<SingleStageRocket> _rocket;

    std::unique_ptr<Environment> _environment;

    Recording _record;

    double _time;

    double _dt;

    double _max_position_error = 1e-3;

    double _max_angular_error = 1e-2;

    double _position_error_mag = 1e-6;

    double _angle_error_proj = 0.999;

    double _min_dt = 1e-5;

    double _max_dt = 0.1;

    void (SingleStageSimulation::*step)();

    void euler_step();

    void huen_step();

    void euler_heun_adaptive_step();

    void rk23_step();

public:

    void set_timestep_constraints(double min_dt, double max_dt);

    void set_error_tolerance(double position_error, double angle_error);

    SingleStageSimulation();
    ~SingleStageSimulation();

    void run(std::string fn, const bool debug = false);

};
