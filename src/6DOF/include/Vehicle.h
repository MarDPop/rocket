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

class SingleStageControl {

    SingleStageRocket& rocket;
public:

    SingleStageControl(SingleStageRocket& r);

};

class WindHistory {

    std::vector<double> times;
    std::vector<Vector> speed;
    double* titer;
    Vector* siter;
    double* tend;
public:

    inline void reset() {
        this->titer = times.data();
        this->siter = speed.data();
        this->tend = this->titer + times.size();
    }

    WindHistory(std::string fn);

    WindHistory(std::vector<double> t, std::vector<Vector> s) : times(t) , speed(s) {
        this->reset();
    }

    inline Vector* get(double time) {
        while(titer < tend && time < *titer) {
            titer++;
            siter++;
        }
        return siter;
    }
};

class SingleStageRocket {

    /**
    * Mass dry (kg)
    */
    double mass_empty;

    /**
    * Mass with propellant (kg)
    */
    double mass_full;

    /**
    * Inertia properties : Ixx, Izz, COG
    */
    double I_empty[3];

    /**
    * change in Inertia vs mass
    */
    double dIdm[3];

    /**
    * Heading to which the rocket z axis is pointed (rad)
    */
    double launch_heading;

    /**
    * The angle away from UP that the z axis of the rocket is tilted (rad)
    */
    double launch_angle = 0;

    /**
    * ground altitude from MSL (m)
    */
    double ground_altitude;

    /**
    * Ground measured pressure (Pa)
    */
    double ground_pressure;

    /**
    * Ground measured temperature (K)
    */
    double ground_temperature;

    /**
    * Lapse rate (K/m), assumed constant for flight (invalid after 10km)
    */
    double lapse_rate;

    /* ATMOSPHERE TABLE */
    // values are precomputed to speed processing
    // values for every meter
    // nearest neighbor is used for interpolation
    std::vector<double> air_density_table;
    std::vector<double> air_pressure_table;
    std::vector<double> air_sound_speed_table;
    std::vector<double> grav_table;

    void init();

    void compute_acceleration();

public:

    static constexpr double R_GAS = 287.052874;

    static constexpr double AIR_CONST = 287.052874*1.4;

    /**
    * current mass  (kg)
    */
    double mass;

    /**
    * current principal moment of inertia cross axially (kg m2)
    */
    double Ixx;

    /**
    * current principal moment of inertia along axis  (kg m2)
    */
    double Izz;

    /**
    * current center of mass location from nose (m)
    */
    double COG;

    /**
    * Current position in ENU (m)
    */
    Vector position;

    /**
    * Current velocity in ENU (m/s)
    */
    Vector velocity;

    /**
    * Current position in ENU (m/s2)
    */
    Vector acceleration;

    /**
    * Current body frame in ENU (z axis is axial)
    */
    Axis CS;

    /**
    * Current angular speed (rad/s)
    */
    Vector angular_velocity;

    /**
    * Current angular acceleration (rad/s)
    */
    Vector angular_acceleration;

    /* USEFUL EXPORTED VALUES */
    /**
    * current gravity (m/s2)
    */
    double grav;

    /**
    * current air pressure (Pa)
    */
    double air_pressure;

    /**
    * current air density (kg/m3)
    */
    double air_density;

    /**
    * current inverse of sound speend  ((m/s) ^ -1)
    */
    double sound_speed_inv;

    /**
    * Internal struct definition to record states
    */
    struct Recording {
        double t_interval = 0.5;
        std::vector<Vector> position;
        std::vector<Axis> orientation;
        std::vector<double> test_value;
    };

    Recording record;

    std::unique_ptr<WindHistory> wind;

    SingleStageAerodynamics aero;

    SingleStageThruster thruster;

    std::unique_ptr<SingleStageControl> control;

    SingleStageRocket();

    SingleStageRocket(const std::string& fn);

    void set_launch(double launch_heading, double launch_angle);

    void set_mass(double empty_mass, double full_mass, double I_empty[3], double I_full[3]);

    void set_ground(double ground_altitude, double ground_pressure, double ground_temperature, double lapse_rate);

    void compute_atmosphere();

    void get_air_properties();

    void launch(double dt);


};

