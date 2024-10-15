#include <imu_aceinna_openimu/Configuration.hpp>
#include <stdexcept>

using namespace std;
using namespace imu_aceinna_openimu;

bool Configuration::operator ==(Configuration const& other) const {
    return !(*this != other);
}

bool Configuration::operator !=(Configuration const& other) const {
    return
        needsReset(other) ||
        other.periodic_packet_rate != periodic_packet_rate ||
        other.periodic_packet_type != periodic_packet_type;
}

bool equal(base::Angle const& a, base::Angle const& b)
{
    return (base::isUnknown(a.getRad()) && base::isUnknown(b.getRad())) || (a == b);
}

bool Configuration::needsReset(Configuration const& original) const
{
    return original.acceleration_low_pass_filter != acceleration_low_pass_filter ||
           original.angular_velocity_low_pass_filter !=
               angular_velocity_low_pass_filter ||
           original.orientation != orientation || original.gps_protocol != gps_protocol ||
           original.gps_baud_rate != gps_baud_rate ||
           original.hard_iron[0] != hard_iron[0] ||
           original.hard_iron[1] != hard_iron[1] ||
           original.soft_iron_ratio != soft_iron_ratio ||
           original.soft_iron_angle != soft_iron_angle ||
           (original.lever_arm - lever_arm).norm() > 1e-3 ||
           (original.point_of_interest - point_of_interest).norm() > 1e-3 ||
           !equal(original.rtk_heading2mag_heading, rtk_heading2mag_heading);
}