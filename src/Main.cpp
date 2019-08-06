#include <iostream>
#include <imu_aceinna_openimu/Driver.hpp>

using namespace std;
using namespace imu_aceinna_openimu;

struct Parameter {
    char const* name;
    int index;
    bool is_integer;
};

static const Parameter PARAMETERS[] = {
    { "periodic-packet-type", 3, false },
    { "periodic-packet-rate", 4, true },
    { "acceleration-filter", 5, true },
    { "angular-velocity-filter", 6, true },
    { nullptr, 0, false }
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

int usage()
{
    cerr
        << "imu_aceinna_openimu_ctl URI [COMMAND] [ARGS]\n"
        << "  URI        a valid iodrivers_base URI, e.g. serial:///dev/ttyUSB0:115200\n"
        << "\n"
        << "Known commands:\n"
        << "  info       display information about the connected unit\n"
        << "  find-rate  find the baud rate on a serial line. Do not specify the rate in the URI\n"
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
        auto conf = driver.readConfiguration();

        cout
            << driver.getDeviceInfo() << "\n"
            << "Periodic packet type: " << conf.periodic_packet_type << "\n"
            << "Periodic packet rate: " << conf.periodic_packet_rate << "\n"
            << "Angular velocity low-pass filter: " << conf.angular_velocity_low_pass_filter << "\n"
            << "Acceleration low-pass filter: " << conf.acceleration_low_pass_filter << "\n"
            << "Orientation: " << conf.orientation << "\n"
            << flush;

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
        else {
            driver.writeConfiguration(definition->index, param_value, true);
        }
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
        cout << driver.getDeviceInfo() << endl;
        return 0;
    }
    else {
        cerr << "unexpected command " << cmd << endl;
        return usage();
    }
}
