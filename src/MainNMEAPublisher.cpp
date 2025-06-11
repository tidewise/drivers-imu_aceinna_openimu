#include <imu_aceinna_openimu/Driver.hpp>
#include <imu_aceinna_openimu/NMEAPublisher.hpp>
#include <iostream>

using namespace std;
using namespace imu_aceinna_openimu;

static void usage(ostream& out)
{
    out << "imu_aceinna_openimu_nmea_publisher DEVICE_URI FORWARD_URI "
           "FORWARD_TIMEOUT NMEA_URI [MESSAGES]\n"
        << "  forwards data (two-way) between URI1 and URI2, which must both\n"
        << "  be valid iodrivers_base URIs. In addition, convert IMU data into NMEA\n"
        << "  sentences and forward them to NMEA_URI\n"
        << "\n"
        << "  FORWARD_TIMEOOUT defines how long (in milliseconds) the forwarder should\n"
        << "  wait on read before forwarding the data, to avoid unnecessary\n"
        << "  fragmentation\n"
        << "\n"
        << "  MESSAGES is a space-separated list of NMEA messages. Supported messages\n"
        << "  are: ZDA, GLL and HDT. The default is HDT for backward-compatibility\n"
        << "  reasons.\n"
        << "\n"
        << flush;
}

static const int BUFFER_SIZE = 32768;

static uint32_t messageNamesToBitset(vector<string> const& message_names)
{
    uint32_t bitset = 0;
    for (auto const& name : message_names) {
        if (name == "HDT") {
            bitset |= NMEA_PUBLISH_HDT;
        }
        else if (name == "ZDA") {
            bitset |= NMEA_PUBLISH_ZDA;
        }
        else if (name == "GLL") {
            bitset |= NMEA_PUBLISH_GLL;
        }
    }
    return bitset;
}

int main(int argc, char** argv)
{
    if (argc < 5) {
        usage(argc == 1 ? cout : cerr);
        return argc == 1 ? 0 : 1;
    }

    uint32_t messages = NMEA_PUBLISH_HDT;
    if (argc > 5) {
        vector<string> message_names(argv + 5, argv + argc);
        messages = messageNamesToBitset(message_names);
    }

    string device_uri = argv[1];
    string forward_uri = argv[2];
    base::Time forward_timeout = base::Time::fromMilliseconds(atoi(argv[3]));
    string nmea_uri = argv[4];

    NMEAPublisher publisher;
    publisher.openDevice(device_uri);
    publisher.openForward(forward_uri, forward_timeout);
    publisher.openNMEA(nmea_uri);
    publisher.selectNMEAMessages(messages);

    while (true) {
        if (!publisher.process(base::Time::fromSeconds(60))) {
            publisher.configureIMU();
        }
    }
    return 0;
}
