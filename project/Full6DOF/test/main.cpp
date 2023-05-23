#include "../src/ode/ODE.h"

#include <cmath>
#include <string>
#include <fstream>
#include <iostream>

#include "../lib/Eigen/Dense"

struct test_dynamics : public virtual ODE_Dynamics<2>
{
    Eigen::Vector2d state;

    bool set_state(const std::array<double, 2>& x, const double& time, std::array<double,2>& dx) override
    {
        memcpy(state.data(), &x[0], 2*sizeof(double));

        dx[0] = x[1];
        dx[1] = -10.0; // - x[1]*fabs(x[1])*0.01;
        return x[0] > -0.0000001;
    }

    double get_position()
    {
        return state[0];
    }

};

void print(const ODE_Recording<2>& recording, std::string filename)
{
    std::ofstream output(filename);

    unsigned nEntries = recording.times.size();
    for(unsigned i = 0; i < nEntries; i++)
    {
        std::array<double,2> state = recording.states[i];
        output << std::to_string(recording.times[i]) << " ";
        output << std::to_string(state[0]) << " " << std::to_string(state[1]) << std::endl;
    }
}

int main(int argc, char** argv) 
{
    std::array<double,2> initial_state = {0.0, 100.0};

    test_dynamics dyn;

    ODE<2> ode(dyn);
    ode.set_timestep(0.01);
    ode.set_state(initial_state);
    ode.run_to_time(10);

    print(ode.get_recording(),"output.dat");

    std::cout << dyn.get_position() << std::endl;

    return 0;
}