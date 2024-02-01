#include <imu_aceinna_openimu/Configuration.hpp>
#include <stdexcept>

using namespace std;
using namespace imu_aceinna_openimu;

static string axisToUser(OrientationAxis axis)
{
    switch (axis) {
        case ORIENTATION_AXIS_PLUS_X:
            return "+X";
        case ORIENTATION_AXIS_MINUS_X:
            return "-X";
        case ORIENTATION_AXIS_PLUS_Y:
            return "+Y";
        case ORIENTATION_AXIS_MINUS_Y:
            return "-Y";
        case ORIENTATION_AXIS_PLUS_Z:
            return "+Z";
        case ORIENTATION_AXIS_MINUS_Z:
            return "-Z";
        default:
            throw std::invalid_argument("axisToUser: invalid axis");
    }
}

Configuration::Orientation::Orientation()
{
}
Configuration::Orientation::Orientation(OrientationAxis forward,
    OrientationAxis right,
    OrientationAxis down)
    : forward(forward)
    , right(right)
    , down(down)
{
}

bool Configuration::Orientation::operator==(Orientation const& other) const
{
    return forward == other.forward && right == other.right && down == other.down;
}

bool Configuration::Orientation::operator!=(Orientation const& other) const
{
    return !(*this == other);
}

string imu_aceinna_openimu::to_string(Configuration::Orientation const& o)
{
    return axisToUser(o.forward) + axisToUser(o.right) + axisToUser(o.down);
}

bool Configuration::needsReset(Configuration const& original) const {
    return
        original.acceleration_low_pass_filter != acceleration_low_pass_filter ||
        original.angular_velocity_low_pass_filter != angular_velocity_low_pass_filter ||
        original.orientation != orientation ||
        original.gps_protocol != gps_protocol ||
        original.gps_baud_rate != gps_baud_rate ||
        original.hard_iron[0] != hard_iron[0] ||
        original.hard_iron[1] != hard_iron[1] ||
        original.soft_iron_ratio != soft_iron_ratio ||
        original.soft_iron_angle != soft_iron_angle ||
        (original.lever_arm - lever_arm).norm() < 1e-3 ||
        (original.point_of_interest - point_of_interest).norm() < 1e-3;
}