#include <fstream>
#include <imu_aceinna_openimu/Driver.hpp>
#include <imu_aceinna_openimu/Parameter.hpp>
#include <imu_aceinna_openimu/Protocol.hpp>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace imu_aceinna_openimu;


struct GPSProtocolDescription {
    GPSProtocol protocol;
    string name;
};

GPSProtocolDescription GPS_PROTOCOLS[] = {
    {          GPS_AUTO,           "auto"},
    {         GPS_UBLOX,          "ublox"},
    {GPS_NOVATEL_BINARY, "novatel-binary"},
    { GPS_NOVATEL_ASCII,  "novatel-ascii"},
    {      GPS_NMEA0183,           "nmea"},
    {   GPS_SIRF_BINARY,    "sirf-binary"}
};

void gpsDisplayProtocolList(ostream& out)
{
    bool first = true;
    for (auto i : GPS_PROTOCOLS) {
        if (!first)
            out << ", ";
        out << i.name;
    }
}

string gpsProtocolToString(GPSProtocol protocol)
{
    for (auto i : GPS_PROTOCOLS) {
        if (protocol == i.protocol) {
            return i.name;
        }
    }
    throw std::invalid_argument("unknown protocol code " + to_string(protocol));
}

GPSProtocol gpsProtocolFromString(string name)
{
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
        << "  poll-pose         periodically display the pose output\n"
        << "  poll-e5           periodically display information about the new E5\n"
        << "                    in addition to all there is in poll-pose, it shows\n"
        << "                    accelerations and temperatures. Requires a TW2 firmware\n"
        << "  covariances       show covariances one time. Requires a TW2 firmware\n"
        << "  poll-mag          periodically display magnetic measurements\n"
        << "  reset             resets the IMU\n"
        << "\n"
        << "  set NAME VALUE    set a configuration parameter. Call without\n"
        << "                    arguments for a list)\n"
        << "  set-period PACKET PERIOD\n"
        << "                    set period of a specific packet. Only effective\n"
        << "                    if periodic-packet-type is set to EP. PERIOD is a\n"
        << "                    number of periods as set by periodic-packet-rate. For\n"
        << "                    instance periodic-packet-rate is 10 and PERIOD is 10,\n"
        << "                    the actual packet period is 1s (100ms base period * 10)\n"
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
            cout << "ID: " << info.device_id << "\n"
                 << "App: " << info.app_version << "\n"
                 << "Periodic packet type: " << conf.periodic_packet_type << "\n"
                 << "Periodic packet rate: " << conf.periodic_packet_rate << "\n"
                 << "Angular velocity low-pass filter: "
                 << conf.angular_velocity_low_pass_filter << "\n"
                 << "Acceleration low-pass filter: " << conf.acceleration_low_pass_filter
                 << "\n"
                 << "Orientation: " << to_string(conf.orientation) << "\n"
                 << "GPS Protocol: " << gpsProtocolToString(conf.gps_protocol) << "\n"
                 << "GPS Baud Rate: " << conf.gps_baud_rate << "\n"
                 << "Hard Iron: " << conf.hard_iron[0] << " " << conf.hard_iron[1] << "\n"
                 << "Soft Iron: ratio=" << conf.soft_iron_ratio << ", "
                 << conf.soft_iron_angle << "\n"
                 << "Lever Arm: "
                 << "x=" << conf.lever_arm.x() << ", "
                 << "y=" << conf.lever_arm.y() << ", "
                 << "z=" << conf.lever_arm.z() << "\n"
                 << "Point of Interest: "
                 << "x=" << conf.point_of_interest.x() << ", "
                 << "y=" << conf.point_of_interest.y() << ", "
                 << "z=" << conf.point_of_interest.z() << "\n"
                 << "RTK Heading to Mag Heading Offset: "
                 << conf.rtk_heading2mag_heading.getDeg() << "\n"
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
            cerr << "set expects exactly two more parameters NAME and VALUE" << endl
                 << endl;
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
        else if (definition->type == PARAM_ANGLE) {
            double written_value = std::stod(param_value);
            driver.writeConfiguration(definition->index, written_value * 180.0 / M_PI, true);
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
    else if (cmd == "poll-pose") {
        int poll_period_usec = 100000;
        if (argc == 4) {
            poll_period_usec = atof(argv[3]) * 1000000;
        }

        driver.openURI(uri);
        driver.validateDevice();
        driver.writePeriodicPacketConfiguration("e2", 10);
        std::cout << "Time Mode Roll Pitch Yaw Lat Lon Alt Vx Vy Vz\n";
        while (true) {
            if (driver.processOne()) {
                auto state = driver.getLastPeriodicUpdate();
                std::cout << state.rbs.time << " " << state.filter_state.toString()
                          << fixed << " " << setprecision(1)
                          << base::getRoll(state.rbs.orientation) * 180 / M_PI << " "
                          << " " << setprecision(1)
                          << base::getPitch(state.rbs.orientation) * 180 / M_PI << " "
                          << " " << setprecision(1)
                          << base::getYaw(state.rbs.orientation) * 180 / M_PI << " "
                          << " " << setprecision(5) << state.latitude.getDeg() << " "
                          << " " << setprecision(5) << state.longitude.getDeg() << " "
                          << " " << setprecision(1) << state.rbs.position.z() << " "
                          << " " << setprecision(2) << state.rbs.velocity.x() << " "
                          << " " << setprecision(2) << state.rbs.velocity.y() << " "
                          << " " << setprecision(2) << state.rbs.velocity.z()
                          << std::endl;
            }

            usleep(poll_period_usec);
        }
    }
    else if (cmd == "covariances") {
        driver.openURI(uri);
        driver.validateDevice();
        driver.writePeriodicPacketConfiguration("e5", 10);
        while (true) {
            if (driver.processOne()) {
                auto state = driver.getLastPeriodicUpdate();
                std::cout << state.rbs.time << " " << state.filter_state.toString()
                          << "\nPosition\n"
                          << " " << setprecision(1) << state.rbs.cov_position
                          << "\nVelocity\n"
                          << " " << setprecision(1) << state.rbs.cov_velocity
                          << "\nQuaternion\n"
                          << " " << setprecision(1) << state.covQuaternion
                          << std::endl;
                break;
            }
        }
    }
    else if (cmd == "poll-e5") {
        int poll_period_usec = 100000;
        if (argc == 4) {
            poll_period_usec = atof(argv[3]) * 1000000;
        }

        driver.openURI(uri);
        driver.validateDevice();
        driver.writePeriodicPacketConfiguration("e5", 10);
        std::cout << "Time Mode Roll Pitch Yaw Lat Lon Alt Vx Vy Vz Ax Ay Az Temp\n";
        while (true) {
            if (driver.processOne()) {
                auto state = driver.getLastPeriodicUpdate();
                std::cout << state.rbs.time << " " << state.filter_state.toString()
                          << fixed << " " << setprecision(1)
                          << base::getRoll(state.rbs.orientation) * 180 / M_PI << " "
                          << " " << setprecision(1)
                          << base::getPitch(state.rbs.orientation) * 180 / M_PI << " "
                          << " " << setprecision(1)
                          << base::getYaw(state.rbs.orientation) * 180 / M_PI << " "
                          << " " << setprecision(5) << state.latitude.getDeg() << " "
                          << " " << setprecision(5) << state.longitude.getDeg() << " "
                          << " " << setprecision(1) << state.rbs.position.z() << " "
                          << " " << setprecision(2) << state.rbs.velocity.x() << " "
                          << " " << setprecision(2) << state.rbs.velocity.y() << " "
                          << " " << setprecision(2) << state.rbs.velocity.z() << " "
                          << " " << setprecision(1) << state.rba.acceleration.x() << " "
                          << " " << setprecision(1) << state.rba.acceleration.y() << " "
                          << " " << setprecision(1) << state.rba.acceleration.z() << " "
                          << " " << setprecision(1) << state.board_temperature.getCelsius()
                          << std::endl;
            }

            usleep(poll_period_usec);
        }
    }
    else if (cmd == "poll-mag") {
        int poll_period_usec = 100000;
        if (argc == 4) {
            poll_period_usec = atof(argv[3]) * 1000000;
        }

        driver.openURI(uri);
        driver.validateDevice();
        driver.writePeriodicPacketConfiguration("e4", 10);
        double const rad2deg = 180.0 / M_PI;
        while (true) {
            if (driver.processOne()) {
                auto state = driver.getLastPeriodicUpdate();
                std::cout << state.magnetic_info.time << fixed << setprecision(2) << " "
                          << state.magnetic_info.magnetometers[0] << " "
                          << state.magnetic_info.magnetometers[1] << " "
                          << state.magnetic_info.magnetometers[2] << " "
                          << atan2(state.magnetic_info.magnetometers[2],
                                 state.magnetic_info.magnetometers[0]) *
                                 rad2deg
                          << " " << state.magnetic_info.measured_euler_angles[0] * rad2deg
                          << " " << state.magnetic_info.measured_euler_angles[1] * rad2deg
                          << " " << state.magnetic_info.measured_euler_angles[2] * rad2deg
                          << " " << state.magnetic_info.declination.getDeg() << std::endl;
            }

            usleep(poll_period_usec);
        }
    }
    else if (cmd == "save-config") {
        driver.openURI(uri);
        driver.validateDevice();
        driver.saveConfiguration();
        std::cout << "Confguration written on permanent storage\n"
                     "Some parameter changes might need a reset to be effective\n";
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
        int rates[] = {38400, 57600, 115200, 230400, 0};
        for (int i = 0;; ++i) {
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
            catch (iodrivers_base::TimeoutError&) {
            }
        }
        auto info = driver.validateDevice();

        cout << "ID: " << info.device_id << "\n"
             << "App: " << info.app_version << std::endl;
        return 0;
    }
    else if (cmd == "to-bootloader") {
        driver.openURI(uri);
        driver.toBootloader();
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
        ifstream file(path, ios::in | ios::binary | ios::ate);
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
                cout << "contacted unit in bootloader mode at " << bootloader_uri << endl;
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
        cout << "ID: " << info.device_id << "\n"
             << "App: " << info.app_version << std::endl;
    }
    else if (cmd == "reset") {
        driver.openURI(uri);
        driver.reset();
        std::cout << "IMU successfully reset" << std::endl;
    }
    else {
        cerr << "unexpected command " << cmd << endl;
        return usage();
    }
}
