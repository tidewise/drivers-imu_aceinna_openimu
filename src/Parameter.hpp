#ifndef IMU_ACEINNA_OPENIMU_PARAMETER_HPP
#define IMU_ACEINNA_OPENIMU_PARAMETER_HPP

#include <ostream>

namespace imu_aceinna_openimu {
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

    inline constexpr Parameter PARAMETERS[] = {
        {"periodic-packet-type", 3, PARAM_STRING, nullptr},
        {"periodic-packet-rate", 4, PARAM_INTEGER, nullptr},
        {"acceleration-filter", 5, PARAM_INTEGER, nullptr},
        {"angular-velocity-filter", 6, PARAM_INTEGER, nullptr},
        {"orientation", 7, PARAM_ORIENTATION, nullptr},
        {"gps-baudrate", 8, PARAM_INTEGER, nullptr},
        {"gps-protocol", 9, PARAM_OTHER, nullptr},
        {"hard-iron-x", 10, PARAM_DOUBLE, nullptr},
        {"hard-iron-y", 11, PARAM_DOUBLE, nullptr},
        {"soft-iron-ratio", 12, PARAM_DOUBLE, nullptr},
        {"soft-iron-angle", 13, PARAM_DOUBLE, nullptr},
        {"lever-arm-x", 14, PARAM_DOUBLE, nullptr},
        {"lever-arm-y", 15, PARAM_DOUBLE, nullptr},
        {"lever-arm-z", 16, PARAM_DOUBLE, nullptr},
        {"point-of-interest-x", 17, PARAM_DOUBLE, nullptr},
        {"point-of-interest-y", 18, PARAM_DOUBLE, nullptr},
        {"point-of-interest-z", 19, PARAM_DOUBLE, nullptr},
        {"rtk-heading-to-mag-heading", 20, PARAM_DOUBLE, "DEGREES"},
        {nullptr, 0, PARAM_OTHER}
    };

    constexpr Parameter const* findParameter(std::string_view const& name)
    {
        for (auto p = PARAMETERS; p->name; ++p) {
            if (p->name == name) {
                return p;
            }
        }
        return nullptr;
    }

    void displayParameters(std::ostream& stream);
}

#endif