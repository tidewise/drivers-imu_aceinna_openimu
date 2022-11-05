#include <imu_aceinna_openimu/Driver.hpp>
#include <imu_aceinna_openimu/NMEAPublisher.hpp>
#include <iostream>

using namespace std;
using namespace imu_aceinna_openimu;

static void usage(ostream& out)
{
    out << "imu_aceinna_openimu_nmea_publisher DEVICE_URI FORWARD_URI "
           "FORWARD_TIMEOUT NMEA_URI\n"
        << "  forwards data (two-way) between URI1 and URI2, which must both\n"
        << "  be valid iodrivers_base URIs. In addition, convert IMU data into NMEA "
           "sentences and forward them to NMEA_URI\n"
        << "\n"
        << "  FORWARD_TIMEOOUT defines how long (in milliseconds) the forwarder "
           "should\n"
        << "  wait on read before forwarding the data, to avoid unnecessary "
           "fragmentation\n"
        << flush;
}

static const int BUFFER_SIZE = 32768;

int main(int argc, char** argv)
{
    if (argc != 5) {
        usage(argc == 1 ? cout : cerr);
        return argc == 1 ? 0 : 1;
    }

    string device_uri = argv[1];
    string forward_uri = argv[2];
    base::Time forward_timeout = base::Time::fromMilliseconds(atoi(argv[3]));
    string nmea_uri = argv[4];

    NMEAPublisher publisher;
    publisher.openDevice(device_uri);
    publisher.openForward(forward_uri, forward_timeout);
    publisher.openNMEA(nmea_uri);

    while (true) {
        if (!publisher.process(base::Time::fromSeconds(60))) {
            publisher.configureIMU();
        }
    }
    return 0;
}
