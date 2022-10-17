#pragma once

#include <vector>
#include <memory>
#include <array>
#include <map>
#include <cmath>
#include <functional>

//#include "../../../lib/Eigen/Dense"

#include "../../common/include/Dynamics.h"
#include "Stage.h"
#include "Body.h"
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

/**
* Standard Vehicle
* Assumptions: Rigid
*/
class Vehicle : public virtual Dynamics<14> {

    unsigned int current_stage_idx;

    std::vector< std::unique_ptr< Stage > > stages;

    Stage* current_stage;

public:

    /**
    * time after launch in seconds
    */
    double TALO;

    /**
    * Standard State of Vehicle
    */
    State state;

    /**
    * total forces acting on vehicle
    */
    Vector force;

    /**
    * total torques acting on vehicle
    */
    Vector moment;

    /**
    * Center of mass location from reference location in vehicle frame
    */
    Vector COM;

    /**
     * Moments of Inertia at Center of Mass
     */
    Inertia inertia;

    /**
    * Coordinate Frame of the Vehicle in inertial frame, this is true of date
    */
    Axis inertial_CS;

    /**
    * Reference of planet in which vehicle is traveling
    */
    Body_Reference body;

    Vehicle();
    virtual ~Vehicle();

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
