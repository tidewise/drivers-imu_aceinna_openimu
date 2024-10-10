#include <pocolog_cpp/SequentialReadDispatcher.hpp>
#include <iodrivers_base/RawPacket.hpp>
#include <iodrivers_base/Driver.hpp>
#include <iostream>
#include <iomanip>
#include <gps_base/BaseTypes.hpp>

using namespace pocolog_cpp;
using namespace std;

void usage() {
    cerr <<
        "usage: imu_aceinna_openimu_gps_replay URI LOGFILE\n"
        "Replays a data stream of type RawPacket into the given stream URI\n"
        "The command assume the log was created by the imu_aceinna_openimu_gps_export tool\n"
        << endl;
}

class Driver : public iodrivers_base::Driver {
    int extractPacket(uint8_t const* buffer, size_t bufferSize) const override {
        return -bufferSize; // this driver is write-only
    }
public:
    using iodrivers_base::Driver::Driver;
};

int main(int argc, char** argv) {
    if (argc != 3) {
        usage();
        return argc != 1;
    }

    string uri = argv[1];
    string logfile_path = argv[2];

    Driver driver(8192);
    driver.openURI(uri);

    base::Time last_t;
    base::Time last_display;

    LogFile logfile(logfile_path);
    SequentialReadDispatcher dispatcher(logfile);
    dispatcher.importTypesFrom("gps_base");
    dispatcher.importTypesFrom("iodrivers_base");
    dispatcher.add<gps_base::Solution>("solution",
        [&last_display](auto const& solution) {
            auto now = base::Time::now();
            if ((now - last_display).toSeconds() > 1) {
                std::cout << solution.time << " "
                          << setw(2) << solution.latitude << " "
                          << setw(2) << solution.longitude << " "
                          << solution.positionType << std::endl;
                last_display = now;
            }
        }
    );
    dispatcher.add<iodrivers_base::RawPacket>("raw_io",
        [&driver, &last_t](auto const& result) {
            if (last_t.isNull()) {
                last_t = result.time;
            }

            usleep((result.time - last_t).toMicroseconds());
            last_t = result.time;

            driver.writePacket(result.data.data(), result.data.size());
        }
    );
    dispatcher.run();
    return 0;
}

