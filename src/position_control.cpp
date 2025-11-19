// Provides control for 1 motor
// Goal is to have motor rotate from its starting position to the commanded position
// Then send a command to have motor go back to its starting position
// Setting up as a test -> will move to multi-motor control once this proof of concept works

#include <iostream>
#include <cmath>
#include <unistd.h> // for usleep
#include "moteus.h"

using namespace std;
using namespace mjbots;

int main(int argc, char** argv) {
    moteus::Controller::DefaultArgProcess(argc, argv);

    moteus::Controller::Options options;
    options.id = 12; // motor ID (set this to your actual ID)
    moteus::Controller c(options);

    cout << "Initializing controller..." << endl;
    c.SetStop(); // clear any faults
    ::usleep(100000); // small delay

    cout << "Sending position commands..." << endl;

    while (true) {
        moteus::PositionMode::Command cmd;

        // if want to set item as NaN, do 'std::numeric_limits<double>::quiet_NaN();'

        cmd.position = 1.0; // units are in revolutions
        cmd.velocity = 0.25; // units are revolutions/s
        cmd.feedforward_torque = 0.0; // give this much extra torque beyond what the normal control loop says
        // gain scaling, keep at configured values for now
        cmd.kp_scale = 1.0;
        cmd.kd_scale = 1.0; 

        const auto maybe_result = c.SetPosition(cmd);

        if (maybe_result) {
            const auto r = maybe_result->values;
            ::printf("%3d p/v/t=(%7.3f,%7.3f,%7.3f)  v/t/f=(%5.1f,%5.1f,%3d)   \r",
                static_cast<int>(r.mode),
                r.position,
                r.velocity,
                r.torque,
                r.voltage,
                r.temperature,
                r.fault);
            ::fflush(stdout);
        }

        ::usleep(20000);
    }

    return 0;
}
