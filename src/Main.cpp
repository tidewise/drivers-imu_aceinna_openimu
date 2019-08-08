#include <iostream>
#include <imu_aceinna_openimu/Driver.hpp>
#include <imu_aceinna_openimu/Protocol.hpp>
#include <fstream>

using namespace std;
using namespace imu_aceinna_openimu;

struct Parameter {
    char const* name;
    int index;
    bool is_integer;
    bool is_orientation;
};

static const Parameter PARAMETERS[] = {
    { "periodic-packet-type", 3, false, false },
    { "periodic-packet-rate", 4, true, false },
    { "acceleration-filter", 5, true, false },
    { "angular-velocity-filter", 6, true, false },
    { "orientation", 7, false, true },
    { nullptr, 0, false, false }
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
    }
}

string gpsProtocolToString(GPSProtocol protocol) {
    switch(protocol) {
        case GPS_AUTO: return "auto";
        case GPS_UBLOX: return "uBlox";
        case GPS_NOVATEL_BINARY: return "Novatel Binary";
        case GPS_NOVATEL_ASCII: return "Novatel ASCII";
        case GPS_NMEA0183: return "NMEA 0183";
        case GPS_SIRF_BINARY: return "SIRF Binary";
        default: return "unknown";
    }
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
        << "\n"
        << "  find-rate         find the baud rate on a serial line. Do not specify\n"
        << "                    the rate in the URI\n"
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
    driver.setReadTimeout(base::Time::fromMilliseconds(100));
    driver.setWriteTimeout(base::Time::fromMilliseconds(100));

    if (cmd == "info") {
        driver.openURI(uri);
        auto info = driver.getDeviceInfo();
        if (driver.isBootloaderMode()) {
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
                << flush;
        }
        return 0;
    }
    else if (cmd == "set") {
        if (argc == 3) {
            cout << "Valid parameters: ";
            displayParameters(cout);
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
        if (definition->is_integer) {
            int64_t written_value = std::stoll(param_value);
            driver.writeConfiguration(definition->index, written_value, true);
        }
        else if (definition->is_orientation) {
            Configuration::Orientation written_value =
                protocol::decodeOrientationString(param_value);
            driver.writeConfiguration(definition->index, written_value, true);
        }
        else {
            driver.writeConfiguration(definition->index, param_value, true);
        }
    }
    else if (cmd == "save-config") {
        driver.openURI(uri);
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
        auto info = driver.getDeviceInfo();

        cout
            << "ID: " << info.device_id << "\n"
            << "App: " << info.app_version << std::endl;
        return 0;
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
        if (!driver.isBootloaderMode()) {
            driver.toBootloader();
            if (uri.find("serial://") == 0) {
                string bootloader_uri = uri.substr(0, uri.rfind(":")) + ":57600";
                driver.openURI(bootloader_uri);
                cout << "contacted unit in bootloader mode at "
                     << bootloader_uri << endl;
                if (!driver.isBootloaderMode()) {
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
        auto info = driver.getDeviceInfo();
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
