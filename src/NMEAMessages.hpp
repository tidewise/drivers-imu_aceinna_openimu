#ifndef IMU_ACEINNA_OPENIMU_NMEAMESSAGES_HPP
#define IMU_ACEINNA_OPENIMU_NMEAMESSAGES_HPP

namespace imu_aceinna_openimu {
    /** Bitmask to select NMEA messages in NMEAPublisher
     */
    enum NMEAMessages {
        NMEA_PUBLISH_HDT = 1,
        NMEA_PUBLISH_ZDA = 2
    };
}

#endif