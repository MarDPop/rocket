#pragma once

#include <vector>
#include <memory>
#include <string>

#include "../../common/include/Cartesian.h"

using namespace Cartesian;

class SingleStageRocket;

class SingleStageThruster {

    std::vector<double> pressures;

    std::vector<double> thrusts;

    std::vector<double> mass_rates;

    int idx = 0;

    int idx_final = 0;

    bool is_constant = true;

    double dT;

    double dM;

public:

    double thrust = 1000;

    double mass_rate = 1;

    void add_thrust_point(double pressure, double thrust, double mass_rate);

    void reset();

    void set(double pressure);

};

struct SingleStageAerodynamics {

    double CD0;

    double CL_alpha;

    double CM_alpha;

    double CM_alpha_dot;

    double K;

    double ref_area;

    double ref_length;

    double stall_angle;

    double CL_max;

    double constant_term;

    double CM_max;

    Vector force;

    Vector moment;

    SingleStageRocket& rocket;

    SingleStageAerodynamics(SingleStageRocket& r);

    void set_coef(double* coef);

    void update();

};

struct Fin {

};

template<unsigned int NFINS>
class SingleStageControl {

    double fin_angle[NFINS]; // first fin on + x axis, going counter clockwise

    double commanded_angle[NFINS]; // first fin on + x axis, going counter clockwise

    Vector fin_direction[NFINS]; // vector of the direction of the span of all fins on Center of pressure

    Vector lift_direction[NFINS]; // vector of the direction of the span of all fins on Center of pressure

    Vector lift_direction[NFINS]; // vector of the direction of the span of all fins on Center of pressure

    double z_location;

    double dCMdTheta;

    double dCDdTheta;

    double dCLdTheta;

    double max_theta = 0.1; // rad

    double slew_rate = 0.5; // rad/s

    double K1;

    double K2;

    double C2;

    double time_old;

    Vector angular_velocity_measured;

    Axis CS_measured;

    SingleStageRocket& rocket;

public:

    Vector dForce;

    Vector dMoment;

    SingleStageControl(SingleStageRocket& r);

    void update(double time);

};

class WindHistory {

    std::vector<double> times;
    std::vector<Vector> speed;
    double* titer;
    Vector* siter;
    double* tend;

    Vector dvdt;

    bool constant = true;

public:

    Vector wind;

    void reset();

    void set(double time);

    WindHistory();

    void load(std::string fn);

    void load(std::vector<double> t, std::vector<Vector> s);
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

    void compute_acceleration(double time);

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
        double t_interval = 0.25;
        std::vector<Vector> position;
        std::vector<Axis> orientation;
        std::vector<double> test_value;
    };

    Recording record;

    WindHistory wind;

    SingleStageAerodynamics aero;

    SingleStageThruster thruster;

    SingleStageControl<3> control;

    SingleStageRocket();

    SingleStageRocket(const std::string& fn);

    void set_launch(double launch_heading, double launch_angle);

    void set_mass(double empty_mass, double full_mass, double I_empty[3], double I_full[3]);

    void set_ground(double ground_altitude, double ground_pressure, double ground_temperature, double lapse_rate);

    void get_inertia();

    void compute_atmosphere();

    void get_air_properties();

    void launch(double dt);


};


