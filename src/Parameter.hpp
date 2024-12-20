#ifndef IMU_ACEINNA_OPENIMU_PARAMETER_HPP
#define IMU_ACEINNA_OPENIMU_PARAMETER_HPP

#include <ostream>

namespace imu_aceinna_openimu {
    enum ParameterType {
        PARAM_STRING,
        PARAM_INTEGER,
        PARAM_DOUBLE,
        PARAM_ANGLE,
        PARAM_OTHER
    };

    struct Parameter {
        char const* name;
        int index;
        ParameterType type = PARAM_OTHER;
        char const* doc;
    };

    enum FirmwareParameterIndex {
        CONF_ORIENTATION = 7,
        CONF_RTK_HEADING_TO_MAG_HEADING = 20
    };

    inline constexpr Parameter PARAMETERS[] = {
        {"periodic-packet-type", 3, PARAM_STRING, nullptr},
        {"periodic-packet-rate", 4, PARAM_INTEGER, nullptr},
        {"acceleration-filter", 5, PARAM_INTEGER, nullptr},
        {"angular-velocity-filter", 6, PARAM_INTEGER, nullptr},
        {"orientation", CONF_ORIENTATION, PARAM_STRING,
            "Mapping between the IMU output axes and its physical axis "
            "(+Y+Z+X maps +X output to +Y physical)"},
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
        {"rtk-heading-to-mag-heading",
         CONF_RTK_HEADING_TO_MAG_HEADING, PARAM_ANGLE,
         "DEGREES, pass nan to disable"},
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
