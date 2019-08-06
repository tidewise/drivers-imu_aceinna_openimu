#include <iostream>
#include <imu_aceinna_openimu/Driver.hpp>

using namespace std;
using namespace imu_aceinna_openimu;

int usage()
{
    std::cerr
        << "imu_aceinna_openimu_ctl URI [COMMAND] [ARGS]\n"
        << "  URI        a valid iodrivers_base URI, e.g. serial:///dev/ttyUSB0:115200\n"
        << "\n"
        << "Known commands:\n"
        << "  info       display information about the connected unit\n"
        << "  find-rate  find the baud rate on a serial line. Do not specify the rate in the URI\n"
        << std::flush;

    return 0;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "not enough arguments" << std::endl;
        return usage();
    }

    string uri(argv[1]);
    string cmd(argv[2]);

    Driver driver;
    driver.setReadTimeout(base::Time::fromMilliseconds(100));
    driver.setWriteTimeout(base::Time::fromMilliseconds(100));

    if (cmd == "info") {
        driver.openURI(uri);
        auto conf = driver.getConfiguration();

        std::cout
            << driver.getDeviceInfo() << "\n"
            << "Periodic packet type: " << conf.periodic_packet_type << "\n"
            << "Periodic packet rate: " << conf.periodic_packet_rate << "\n"
            << "Angular velocity low-pass filter: " << conf.angular_velocity_low_pass_filter << "\n"
            << "Acceleration low-pass filter: " << conf.acceleration_low_pass_filter << "\n"
            << "Orientation: " << conf.orientation << "\n"
            << std::flush;

        return 0;
    }
    else if (cmd == "find-rate") {
        int rates[] = { 38400, 57600, 115200, 230400, 0 };
        for (int i = 0; ; ++i) {
            int r = rates[i];
            if (r == 0) {
                std::cerr << "cannot find openIMU on " + uri << std::endl;
                return 1;
            }

            std::cout << "trying " << r << std::endl;
            try {
                driver.openURI(uri + ":" + std::to_string(r));
                std::cout << "found OpenIMU at " << r << " bauds" << std::endl;
                break;
            }
            catch(iodrivers_base::TimeoutError&) {}
        }
        std::cout << driver.getDeviceInfo() << std::endl;
        return 0;
    }
    else {
        std::cerr << "unexpected command " << cmd << std::endl;
        return usage();
    }
}
