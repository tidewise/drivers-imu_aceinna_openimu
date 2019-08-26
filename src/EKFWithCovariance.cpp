#include <imu_aceinna_openimu/EKFWithCovariance.hpp>

using namespace std;
using namespace imu_aceinna_openimu;

void EKFWithCovariance::computeNWUPosition(gps_base::UTMConverter const& converter)
{
    gps_base::Solution latlonalt;
    latlonalt.latitude = latitude.getDeg();
    latlonalt.longitude = longitude.getDeg();
    latlonalt.altitude = rbs.position.z();
    latlonalt.positionType = gps_base::AUTONOMOUS;

    auto nwu = converter.convertToNWU(latlonalt);
    rbs.position = nwu.position;
}
