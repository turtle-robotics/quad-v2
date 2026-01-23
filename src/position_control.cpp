// Provides control for 1 motor
// Goal is to have motor rotate from its starting position to the commanded position
// Then send a command to have motor go back to its starting position
// Setting up as a test -> will move to multi-motor control once this proof of concept works

#include <iostream>
#include <cmath>
#include <unistd.h> // for usleep
#include <signal.h>
#include "moteus.h"
#include "pi3hat_moteus_transport.h"

using namespace std;
using namespace mjbots;
using Transport = pi3hat::Pi3HatMoteusTransport;
void signal_callback_handler(int signum)
{
    ::printf("\033[2J\033[H"); // clear screen

    exit(signum);
}

int main(int argc, char **argv)
{

    signal(SIGINT, signal_callback_handler);

    moteus::Controller::DefaultArgProcess(argc, argv);

    Transport::Options toptions;

    for (int i = 1; i < 5; i++) // leg number
    {
        for (int j = 1; j < 4; j++) // joint number in leg
        {
            toptions.servo_map[i * 10 + j] = i; // map servo ID to bus number
        }
    }
    auto transport = make_shared<Transport>(toptions);

    cout << "Creating controller..." << endl;
    moteus::Controller *c[12];

    for (int i = 1; i < 5; i++)
    {
        for (int j = 1; j < 4; j++)
        {
            c[(i - 1) * 3 + (j - 1)] = new moteus::Controller([&]()
                                                              {
    moteus::Controller::Options coptions;
    coptions.id=i*10+j;
    coptions.transport = transport;
    return coptions; }());
        }
    }

    cout << "Initializing controller..." << endl;
    for (int i = 0; i < 12; i++)
        c[i]->SetStop(); // clear any faults

    ::usleep(100000); // small delay

    cout << "Sending position commands..." << endl;

    int min_id = 0;
    int max_id = 11;
    int n_ids = max_id - min_id + 1;

    while (true)
    {
        moteus::PositionMode::Command cmd;

        // if want to set item as NaN, do 'std::numeric_limits<double>::quiet_NaN();'

        cmd.position = 0.0;                                      // units are in revolutions
        cmd.velocity = std::numeric_limits<double>::quiet_NaN(); // units are revolutions/s
        cmd.feedforward_torque = 0.0;                            // give this much extra torque beyond what the normal control loop says
        // gain scaling, keep at configured values for now
        cmd.kp_scale = 1.0;
        cmd.kd_scale = 1.0;
        int setzero[] = {11, 12, 13, 31, 32, 33};
        moteus::Query::Result r[12];

        for (int i = 0; i < 12; i++)
        {
            for (int id : setzero)
                if (c[i]->options().id == id)
                {
                    const auto maybe_result = c[i]->SetPosition(cmd);
                    if (maybe_result)
                        r[i] = maybe_result->values;
                    break;
                }
            const auto maybe_result = c[i]->SetQuery();

            if (maybe_result)
            {
                const moteus::Query::Result r = maybe_result->values;

                ::printf("%2d: %3d p/v/t=(%7.3f,%7.3f,%7.3f)  v/t/f=(%5.1f,%5.1f,%3d)   \n",
                         c[i]->options().id,
                         static_cast<int>(r.mode),
                         r.position,
                         r.velocity,
                         r.torque,
                         r.voltage,
                         r.temperature,
                         r.fault);
            }
            else
            {
                ::printf("%2d:  failed to return result                                \n",
                         c[i]->options().id);
            }
        }
        ::printf("\033[%dA", 12);
        ::fflush(stdout);
        ::usleep(50000);
    }

    for (int i = 0; i < 12; i++)
        c[i]->SetStop();

    return 0;
}
