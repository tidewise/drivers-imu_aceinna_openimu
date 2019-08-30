#include <imu_aceinna_openimu/Configuration.hpp>
#include <stdexcept>

using namespace std;
using namespace imu_aceinna_openimu;

static string axisToUser(ORIENTATION_AXIS axis) {
    switch(axis) {
        case ORIENTATION_AXIS_PLUS_X: return "+X";
        case ORIENTATION_AXIS_MINUS_X: return "-X";
        case ORIENTATION_AXIS_PLUS_Y: return "+Y";
        case ORIENTATION_AXIS_MINUS_Y: return "-Y";
        case ORIENTATION_AXIS_PLUS_Z: return "+Z";
        case ORIENTATION_AXIS_MINUS_Z: return "-Z";
        default:
            throw std::invalid_argument("axisToUser: invalid axis");
    }
}

Configuration::Orientation::Orientation() {
}
Configuration::Orientation::Orientation(ORIENTATION_AXIS forward, ORIENTATION_AXIS right, ORIENTATION_AXIS down)
    : forward(forward)
    , right(right)
    , down(down) {
}

bool Configuration::Orientation::operator ==(Orientation const& other) const
{
    return forward == other.forward &&
           right == other.right &&
           down == other.down;
}

bool Configuration::Orientation::operator !=(Orientation const& other) const
{
    return !(*this == other);
}

string imu_aceinna_openimu::to_string(Configuration::Orientation const& o)
{
    return axisToUser(o.forward) + axisToUser(o.right) + axisToUser(o.down);
}
