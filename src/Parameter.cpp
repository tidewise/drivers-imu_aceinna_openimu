#include <imu_aceinna_openimu/Parameter.hpp>

using namespace imu_aceinna_openimu;

void imu_aceinna_openimu::displayParameters(std::ostream& stream)
{
    for (auto p = PARAMETERS; p->name; ++p) {
        stream << "\n" << p->name;
        if (p->doc) {
            stream << ": " << p->doc;
        }
    }
}