#include <imu_aceinna_openimu/FilterState.hpp>

using namespace std;
using namespace imu_aceinna_openimu;

string FilterState::toString() const
{
    string result;
    if (mode == OPMODE_STABILIZING) {
        result = "STABILIZING";
    }
    else if (mode == OPMODE_INITIALIZING) {
        result = "INITIALIZING";
    }
    else if (mode == OPMODE_AHRS_LOW_GAIN) {
        result = "AHRS_LOW_GAIN";
    }
    else if (mode == OPMODE_AHRS_HIGH_GAIN) {
        result = "AHRS_HIGH_GAIN";
    }
    else if (mode == OPMODE_INS) {
        result = "INS";
    }
    else {
        result = "UNKNOWN(" + to_string(mode) + ")";
    }

    if (status & LINEAR_ACCELERATION) {
        result += "|LINEAR_ACCELERATION";
    }
    if (status & TURN_SWITCH) {
        result += "|TURN_SWITCH";
    }
    if (status & COURSE_USED_AS_HEADING) {
        result += "|COURSE_USED_AS_HEADING";
    }

    return result;
}
