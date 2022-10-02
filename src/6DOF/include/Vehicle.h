#pragma once

#include <vector>
#include <memory>
#include <array>
#include <cmath>
#include <functional>

#include "../../common/include/Dynamics.h"
#include "Stage.h"
#include "Planet.h"
#include "../../common/include/Cartesian.h"

using namespace Cartesian;

struct State {
    std::array<double,14> x;
    double* const position;
    double* const velocity;
    double* const attitude;
    double* const angular_velocity;
    double* const mass;

    State() : position(&x[0]), velocity(&x[3]), attitude(&x[6]), angular_velocity(&x[10]), mass(&x[13]) { }
};

typedef std::array<double,6> Inertia;

class Vehicle : public virtual Dynamics<14> {
    unsigned int current_stage_idx;

    std::vector< std::unique_ptr< Stage > > stages;

    Stage* current_stage;

public:

    double TALO;

    /**
    * Standard State of Vehicle
    */
    State state;

    Vector force;

    Vector moment;

    Vector COM;

    /**
     * Moments of Inertia at Center of Mass
     */
    Inertia inertia;

    /**
    * Coordinate Frame of the Vehicle in inertial frame
    */
    Axis inertial_CS;

    /**
    * Reference of planet in which vehicle is traveling
    */
    Planet planet;

    Vehicle();
    virtual ~Vehicle();

    void update(double time);

    void set_orientation(double* q);

    void get_state_rate(std::array<double,14>& x, double t, std::array<double,14>& dx) override;

    std::function<void(double*,double*)> compute_moment;

    void compute_no_moment(double* w, double* dw);

    void compute_symmetric_moment(double* w, double* dw);

    void compute_plane_moment(double* w, double* dw);

    void compute_full_moment(double* w, double* dw);

    inline void add_stage(Stage* stage) {
        stages.emplace_back(stage);
        stage->set_vehicle(this);
    }

    inline void set_stage(unsigned int stage_idx){
        if(stage_idx < this->stages.size()) {
            this->current_stage_idx = stage_idx;
            this->current_stage = this->stages[stage_idx].get();

            if(this->current_stage->is_symmetric){
                this->compute_moment = std::bind(&Vehicle::compute_symmetric_moment,this,std::placeholders::_1,std::placeholders::_2);
                return;
            }
            if(this->current_stage->is_plane){
                this->compute_moment = std::bind(&Vehicle::compute_plane_moment,this,std::placeholders::_1,std::placeholders::_2);
                return;
            }

            if(this->current_stage->is_ballistic || this->current_stage->is_3DOF) {
                this->compute_moment = std::bind(&Vehicle::compute_no_moment,this,std::placeholders::_1,std::placeholders::_2);
                return;
            }

            this->compute_moment = std::bind(&Vehicle::compute_full_moment,this,std::placeholders::_1,std::placeholders::_2);
        }
    }

};


class Vehicle_Component {

    std::vector< Component > components;

public:

    Vehicle_Component();

};

class Vehicle_Simplified {

    Vector position;

    Vector velocity;

    Vector acceleration;

    Axis CS;

public:

    std::unique_ptr< Aerodynamics > aero;

    std::unique_ptr< Thruster > thruster;

};

