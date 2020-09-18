#include <iostream>
#include <imu_aceinna_openimu/Driver.hpp>
#include <imu_aceinna_openimu/Protocol.hpp>
#include <fstream>
#include <iomanip>

using namespace std;
using namespace imu_aceinna_openimu;

enum ParameterType {
    PARAM_STRING,
    PARAM_INTEGER,
    PARAM_ORIENTATION,
    PARAM_DOUBLE,
    PARAM_OTHER
};

struct Parameter {
    char const* name;
    int index;
    ParameterType type = PARAM_OTHER;
    char const* doc;
};

static const Parameter PARAMETERS[] = {
    { "periodic-packet-type", 3, PARAM_STRING, nullptr },
    { "periodic-packet-rate", 4, PARAM_INTEGER, nullptr },
    { "acceleration-filter", 5, PARAM_INTEGER, nullptr },
    { "angular-velocity-filter", 6, PARAM_INTEGER, nullptr },
    { "orientation", 7, PARAM_ORIENTATION, nullptr },
    { "gps-baudrate", 8, PARAM_INTEGER, nullptr },
    { "gps-protocol", 9, PARAM_OTHER, nullptr },
    { "hard-iron-x", 10, PARAM_DOUBLE, nullptr },
    { "hard-iron-y", 11, PARAM_DOUBLE, nullptr },
    { "soft-iron-ratio", 12, PARAM_DOUBLE, nullptr },
    { "soft-iron-angle", 13, PARAM_DOUBLE, nullptr },
    { "lever-arm-x", 14, PARAM_DOUBLE, nullptr },
    { "lever-arm-y", 15, PARAM_DOUBLE, nullptr },
    { "lever-arm-z", 16, PARAM_DOUBLE, nullptr },
    { "point-of-interest-x", 17, PARAM_DOUBLE, nullptr },
    { "point-of-interest-y", 18, PARAM_DOUBLE, nullptr },
    { "point-of-interest-z", 19, PARAM_DOUBLE, nullptr },
    { nullptr, 0, PARAM_OTHER }
};

Parameter const* findParameter(string name) {
    for (auto p = PARAMETERS; p->name; ++p) {
        if (p->name == name) {
            return p;
        }
    }
    return nullptr;
}

void displayParameters(ostream& stream) {
    for (auto p = PARAMETERS; p->name; ++p) {
        stream << "\n" << p->name;
        if (p->doc) {
            stream << ": " << p->doc;
        }
    }
}

struct GPSProtocolDescription {
    GPSProtocol protocol;
    string name;
};

GPSProtocolDescription GPS_PROTOCOLS[] = {
    { GPS_AUTO, "auto" },
    { GPS_UBLOX, "ublox" },
    { GPS_NOVATEL_BINARY, "novatel-binary" },
    { GPS_NOVATEL_ASCII, "novatel-ascii" },
    { GPS_NMEA0183, "nmea" },
    { GPS_SIRF_BINARY, "sirf-binary" }
};

void gpsDisplayProtocolList(ostream& out) {
    bool first = true;
    for (auto i : GPS_PROTOCOLS) {
        if (!first)
            out << ", ";
        out << i.name;
    }
}

string gpsProtocolToString(GPSProtocol protocol) {
    for (auto i : GPS_PROTOCOLS) {
        if (protocol == i.protocol) {
            return i.name;
        }
    }
    throw std::invalid_argument("unknown protocol code " + to_string(protocol));
}

GPSProtocol gpsProtocolFromString(string name) {
    for (auto i : GPS_PROTOCOLS) {
        if (name == i.name) {
            return i.protocol;
        }
    }
    throw std::invalid_argument("unknown protocol name " + name);
}

int usage()
{
    cerr
        << "imu_aceinna_openimu_ctl URI [COMMAND] [ARGS]\n"
        << "  URI        a valid iodrivers_base URI, e.g. serial:///dev/ttyUSB0:115200\n"
        << "\n"
        << "Known commands:\n"
        << "  info              display information about the connected unit\n"
        << "  set NAME VALUE    set a configuration parameter. Call without\n"
        << "                    arguments for a list)\n"
        << "  sensor NAME ENABLED enable (ENABLED='on') or disable (ENABLED='off')\n"
        << "                    a particular sensor. Run without arguments for a list\n"
        << "\n"
        << "  find-rate         find the baud rate on a serial line. Do not specify\n"
        << "                    the rate in the URI\n"
        << "  set-period PACKET PERIOD set period of a specific packet. Only effective\n"
        << "                    if periodic-packet-type is set to EP. PERIOD is a number of\n"
        << "                    periods as set by periodic-packet-rate. For instance, if\n"
        << "                    periodic-packet-rate is 10 and PERIOD is 10, the actual\n"
        << "                    packet period is 1s (100ms base period * 10)\n"
        << "  set-rate RATE     change the baud rate. The new rate will be effective\n"
        << "                    only after a save-config and a reset\n"
        << "  save-config       save the current configuration to flash\n"
        << "\n"
        << "  to-bootloader     switch to bootloader mode. Communication when in\n"
        << "                    bootloader mode is always using 57600 bauds\n"
        << "  to-app            switch to app mode\n"
        << "  write-firmware P  write a firmware file\n"
        << flush;

    return 0;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        cerr << "not enough arguments" << endl;
        return usage();
    }

    string uri(argv[1]);
    string cmd(argv[2]);

    Driver driver;
    driver.setReadTimeout(base::Time::fromMilliseconds(1000));
    driver.setWriteTimeout(base::Time::fromMilliseconds(1000));

    if (cmd == "info") {
        driver.openURI(uri);
        auto info = driver.validateDevice(true);
        if (info.bootloader_mode) {
            cout << "ID: " << info.device_id << std::endl;
        }
        else {
            auto conf = driver.readConfiguration();
            cout
                << "ID: " << info.device_id << "\n"
                << "App: " << info.app_version << "\n"
                << "Periodic packet type: " << conf.periodic_packet_type << "\n"
                << "Periodic packet rate: " << conf.periodic_packet_rate << "\n"
                << "Angular velocity low-pass filter: " << conf.angular_velocity_low_pass_filter << "\n"
                << "Acceleration low-pass filter: " << conf.acceleration_low_pass_filter << "\n"
                << "Orientation: " << to_string(conf.orientation) << "\n"
                << "GPS Protocol: " << gpsProtocolToString(conf.gps_protocol) << "\n"
                << "GPS Baud Rate: " << conf.gps_baud_rate << "\n"
                << "Hard Iron: " << conf.hard_iron[0] << " " << conf.hard_iron[1] << "\n"
                << "Soft Iron: ratio=" << conf.soft_iron_ratio << ", " << conf.soft_iron_angle << "\n"
                << "Lever Arm: "
                    << "x=" << conf.lever_arm.x() << ", "
                    << "y=" << conf.lever_arm.y() << ", "
                    << "z=" << conf.lever_arm.z() << "\n"
                << "Point of Interest: "
                    << "x=" << conf.point_of_interest.x() << ", "
                    << "y=" << conf.point_of_interest.y() << ", "
                    << "z=" << conf.point_of_interest.z() << "\n"
                << flush;
        }
        return 0;
    }
    else if (cmd == "set") {
        if (argc == 3) {
            cout << "Valid parameters: ";
            displayParameters(cout);
            cout << std::endl;
            return 0;
        }
        else if (argc != 5) {
            cerr << "set expects exactly two more parameters NAME and VALUE" << endl << endl;
            return usage();
        }

        string param_name = argv[3];
        string param_value = argv[4];
        auto definition = findParameter(param_name);
        if (!definition) {
            cerr << "Unknown or read-only parameter '" << param_name << "', "
                 << "known parameters are: ";
            displayParameters(cerr);
            return 1;
        }

        driver.openURI(uri);
        driver.validateDevice();
        if (param_name == "gps-protocol") {
            GPSProtocol protocol = gpsProtocolFromString(param_value);
            driver.writeGPSProtocol(protocol);
        }
        else if (definition->type == PARAM_INTEGER) {
            int64_t written_value = std::stoll(param_value);
            driver.writeConfiguration(definition->index, written_value, true);
        }
        else if (definition->type == PARAM_DOUBLE) {
            double written_value = std::stod(param_value);
            driver.writeConfiguration(definition->index, written_value, true);
        }
        else if (definition->type == PARAM_ORIENTATION) {
            Configuration::Orientation written_value =
                protocol::decodeOrientationString(param_value);
            driver.writeConfiguration(definition->index, written_value, true);
        }
        else if (definition->type == PARAM_STRING) {
            driver.writeConfiguration(definition->index, param_value, true);
        }
        else {
            throw std::invalid_argument("do not know how to set parameter " + param_name);
        }
    }
    else if (cmd == "show-packets") {
        driver.openURI(uri);
        driver.validateDevice();
        while(true) {
            auto updated = driver.processOne();
            cout << base::Time::now() << " ";
            if (updated.isUpdated(Driver::UPDATED_RAW_IMU_SENSORS)) {
                cout << " " << "RAW_IMU_SENSORS";
            }
            if (updated.isUpdated(Driver::UPDATED_STATE)) {
                cout << " " << "STATE";
            }
            if (updated.isUpdated(Driver::UPDATED_STATUS)) {
                cout << " " << "STATUS";
            }
            cout << std::endl;
        }
    }
    else if (cmd == "poll") {
        int poll_period_usec = 100000;
        if (argc == 4) {
            poll_period_usec = atof(argv[3]) * 1000000;
        }

        driver.openURI(uri);
        driver.validateDevice();
        driver.writePeriodicPacketConfiguration("e2", 10);
        while(true) {
            if (driver.processOne().isUpdated(Driver::UPDATED_STATE)) {
                auto state = driver.getState();
                std::cout << state.rbs.time
                        << " " << state.filter_state.toString()
                        << fixed
                        << " " << setprecision(1) << base::getRoll(state.rbs.orientation) * 180 / M_PI << " "
                        << " " << setprecision(1) << base::getPitch(state.rbs.orientation) * 180 / M_PI << " "
                        << " " << setprecision(1) << base::getYaw(state.rbs.orientation) * 180 / M_PI << " "
                        << " " << setprecision(1) << state.rbs.position.x() << " "
                        << " " << setprecision(1) << state.rbs.position.y() << " "
                        << " " << setprecision(1) << state.rbs.position.z() << " "
                        << " " << setprecision(1) << state.rbs.velocity.x() << " "
                        << " " << setprecision(1) << state.rbs.velocity.y() << " "
                        << " " << setprecision(1) << state.rbs.velocity.z() << std::endl;
            }

            usleep(poll_period_usec);
        }
    }
    else if (cmd == "save-config") {
        driver.openURI(uri);
        driver.validateDevice();
        driver.saveConfiguration();
    }
    else if (cmd == "set-rate") {
        if (argc != 4) {
            std::cerr << "set-rate expectes a single RATE argument\n" << std::endl;
            usage();
            return 0;
        }
        int rate = stoll(argv[3]);
        cout << "Changing baud rate to " << rate << std::endl;
        driver.openURI(uri);
        driver.validateDevice();
        driver.writeBaudrate(stoll(argv[3]));
        driver.saveConfiguration();
        cout << "You need to restart the IMU for the change to take effect" << std::endl;
        return 0;
    }
    else if (cmd == "find-rate") {
        int rates[] = { 38400, 57600, 115200, 230400, 0 };
        for (int i = 0; ; ++i) {
            int r = rates[i];
            if (r == 0) {
                cerr << "cannot find openIMU on " + uri << endl;
                return 1;
            }

            cout << "trying " << r << endl;
            try {
                driver.openURI(uri + ":" + to_string(r));
                cout << "found OpenIMU at " << r << " bauds" << endl;
                break;
            }
            catch(iodrivers_base::TimeoutError&) {}
        }
        auto info = driver.validateDevice();

        cout
            << "ID: " << info.device_id << "\n"
            << "App: " << info.app_version << std::endl;
        return 0;
    }
    else if (cmd == "to-app") {
        driver.openURI(uri);
        driver.toApp();
    }
    else if (cmd == "write-firmware") {
        if (argc != 4) {
            cerr << "expected a single argument FILE\n" << std::endl;
            usage();
            return 1;
        }

        std::vector<uint8_t> firmware;
        string path = argv[3];
        ifstream file(path, ios::in|ios::binary|ios::ate);
        if (!file.is_open()) {
            throw std::runtime_error("cannot open " + path + " for reading");
        }

        int size = file.tellg();
        firmware.resize(size);
        file.seekg(0, ios::beg);
        file.read(reinterpret_cast<char*>(&firmware[0]), size);
        file.close();

        driver.openURI(uri);
        auto info = driver.validateDevice(true);
        if (!info.bootloader_mode) {
            driver.toBootloader();
            if (uri.find("serial://") == 0) {
                string bootloader_uri = uri.substr(0, uri.rfind(":")) + ":57600";
                driver.openURI(bootloader_uri);
                info = driver.readDeviceInfo();
                cout << "contacted unit in bootloader mode at "
                     << bootloader_uri << endl;
                if (!info.bootloader_mode) {
                    std::cerr << "failed to switch to bootloader mode" << std::endl;
                    return 1;
                }
            }
            else {
                std::cout << "WARN: not accessing the unit through a serial line\n"
                          << "WARN: reopening the device in bootloader mode might fail"
                          << std::endl;
                driver.openURI(uri);
            }
        }

        driver.writeFirmware(firmware, std::cout);
        driver.toApp();
        sleep(5); // from the python code
        driver.openURI(uri);
        cout << "\rfirmware update successful" << endl;
        info = driver.readDeviceInfo();
        cout
            << "ID: " << info.device_id << "\n"
            << "App: " << info.app_version << std::endl;
    }
    else if (cmd == "reset") {
        driver.openURI(uri);
        driver.queryReset();
    }
    else {
        cerr << "unexpected command " << cmd << endl;
        return usage();
    }
}
