#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <imu_aceinna_openimu/Protocol.hpp>

using namespace std;
using namespace imu_aceinna_openimu;
using namespace imu_aceinna_openimu::protocol;
using testing::ElementsAre;
using testing::ElementsAreArray;

#define ASSERT_THROW_MESSAGE(code, exception, message)                                   \
    ASSERT_THROW(                                                                        \
        {                                                                                \
            try {                                                                        \
                code;                                                                    \
            }                                                                            \
            catch (exception & e) {                                                      \
                ASSERT_EQ(message, string(e.what()));                                    \
                throw;                                                                   \
            }                                                                            \
        },                                                                               \
        exception)

struct ProtocolTest : public ::testing::Test {
    std::vector<uint8_t> buffer;

    ProtocolTest()
    {
        buffer.resize(256, 0);
    }
};

TEST_F(ProtocolTest, it_waits_for_more_bytes_if_the_buffer_is_too_small)
{
    ASSERT_EQ(0, extractPacket(&buffer[0], 4));
}
TEST_F(ProtocolTest, it_finds_a_packet_start_within_the_packet)
{
    buffer[5] = PACKET_START_MARKER;
    buffer[6] = PACKET_START_MARKER;
    ASSERT_EQ(-5, extractPacket(&buffer[0], 9));
}
TEST_F(ProtocolTest, it_handles_a_start_byte_at_the_very_end_of_the_buffer)
{
    buffer[9] = PACKET_START_MARKER;
    ASSERT_EQ(-9, extractPacket(&buffer[0], 10));
}
TEST_F(ProtocolTest, it_rejects_a_packet_with_an_invalid_CRC)
{
    buffer[0] = PACKET_START_MARKER;
    buffer[1] = PACKET_START_MARKER;
    ASSERT_EQ(-1, extractPacket(&buffer[0], 10));
}
TEST_F(ProtocolTest, it_accepts_a_packet_that_starts_at_zero_and_has_a_valid_CRC)
{
    buffer = {PACKET_START_MARKER, PACKET_START_MARKER, 'p', 'G', 0, 0x5d, 0x5f};
    ASSERT_EQ(7, extractPacket(&buffer[0], 10));
}

TEST_F(ProtocolTest, it_formats_a_device_id_query)
{
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    auto packetEnd = queryDeviceID(&buffer[0]);

    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd),
        ElementsAre(PACKET_START_MARKER, PACKET_START_MARKER, 'p', 'G', 0, 0x5d, 0x5f));
}

TEST_F(ProtocolTest, it_parses_a_device_id_response)
{
    std::vector<uint8_t> buffer = {'a', 'b', 'C', 'd'};

    ASSERT_EQ("abCd", parseDeviceID(&buffer[0], 4));
}

TEST_F(ProtocolTest, it_formats_an_app_version_query)
{
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    auto packetEnd = queryAppVersion(&buffer[0]);

    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd),
        ElementsAre(PACKET_START_MARKER, PACKET_START_MARKER, 'g', 'V', 0, 0xab, 0xee));
}

TEST_F(ProtocolTest, it_parses_an_app_version_response)
{
    std::vector<uint8_t> buffer = {'a', 'b', 'C', 'd'};

    ASSERT_EQ("abCd", parseAppVersion(&buffer[0], 4));
}

TEST_F(ProtocolTest, it_formats_a_configuration_query)
{
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    auto packetEnd = queryConfiguration(&buffer[0]);

    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd),
        ElementsAre(PACKET_START_MARKER, PACKET_START_MARKER, 'g', 'A', 0, 0x31, 0x0A));
}

TEST_F(ProtocolTest, it_parses_a_configuration_response)
{
    std::vector<uint8_t> buffer = {
        1,
        0,
        0,
        0,
        0,
        0,
        0,
        0, // Data CRC
        2,
        0,
        0,
        0,
        0,
        0,
        0,
        0, // Data Size
        3,
        0,
        0,
        0,
        0,
        0,
        0,
        0, // Baud Rate
        'g',
        'A',
        ' ',
        ' ',
        ' ',
        ' ',
        ' ',
        ' ', // Periodic Packet Type
        4,
        0,
        0,
        0,
        0,
        0,
        0,
        0, // Periodic Packet Rate
        5,
        0,
        0,
        0,
        0,
        0,
        0,
        0, // Accel low-pass filter
        6,
        0,
        0,
        0,
        0,
        0,
        0,
        0, // Rate low-pass filter
        '+',
        'X',
        '-',
        'Y',
        '+',
        'Z',
        ' ',
        ' ', // Orientation
        10,
        0,
        0,
        0,
        0,
        0,
        0,
        0, // GPS Baudrate
        1,
        0,
        0,
        0,
        0,
        0,
        0,
        0, // GPS Protocol
        0,
        0,
        0,
        0,
        0,
        0,
        0x1C,
        0x40, // Hard Iron X
        0,
        0,
        0,
        0,
        0,
        0,
        0x20,
        0x40, // Hard Iron Y
        0,
        0,
        0,
        0,
        0,
        0,
        0x22,
        0x40, // Soft Iron Ratio
        0,
        0,
        0,
        0,
        0,
        0,
        0x24,
        0x40, // Soft Iron Angle
        0,
        0,
        0,
        0,
        0,
        0,
        0xf0,
        0x3f, // Lever Arm X
        0,
        0,
        0,
        0,
        0,
        0,
        0x00,
        0x40, // Lever Arm Y
        0,
        0,
        0,
        0,
        0,
        0,
        0x08,
        0x40, // Lever Arm Z
        0,
        0,
        0,
        0,
        0,
        0,
        0x10,
        0x40, // Point of Interest X
        0,
        0,
        0,
        0,
        0,
        0,
        0x14,
        0x40, // Point of Interest Y
        0,
        0,
        0,
        0,
        0,
        0,
        0x18,
        0x40, // Point of Interest Z
        0,
        0,
        0,
        0,
        0,
        0x80,
        0x56,
        0x40, // rtk_heading2mag_heading -> 90.0
    };

    auto conf = parseConfiguration(&buffer[0], buffer.size());
    ASSERT_EQ("gA", conf.periodic_packet_type);
    ASSERT_EQ(4, conf.periodic_packet_rate);
    ASSERT_EQ(5, conf.acceleration_low_pass_filter);
    ASSERT_EQ(6, conf.angular_velocity_low_pass_filter);
    ASSERT_EQ("+X-Y+Z", conf.orientation);
    ASSERT_EQ(10, conf.gps_baud_rate);
    ASSERT_EQ(1, conf.gps_protocol);
    ASSERT_FLOAT_EQ(7, conf.hard_iron[0]);
    ASSERT_FLOAT_EQ(8, conf.hard_iron[1]);
    ASSERT_FLOAT_EQ(9, conf.soft_iron_ratio);
    ASSERT_FLOAT_EQ(10, conf.soft_iron_angle);
    ASSERT_FLOAT_EQ(1, conf.lever_arm.x());
    ASSERT_FLOAT_EQ(2, conf.lever_arm.y());
    ASSERT_FLOAT_EQ(3, conf.lever_arm.z());
    ASSERT_FLOAT_EQ(4, conf.point_of_interest.x());
    ASSERT_FLOAT_EQ(5, conf.point_of_interest.y());
    ASSERT_FLOAT_EQ(6, conf.point_of_interest.z());
    ASSERT_FLOAT_EQ(90, conf.rtk_heading2mag_heading.getDeg());
}

TEST_F(ProtocolTest,
    it_raises_if_configuration_buffer_is_smaller_than_the_expected_struct)
{
    ASSERT_THROW(parseConfiguration(nullptr, 55), std::invalid_argument);
}

TEST_F(ProtocolTest, it_formats_a_configuration_write_for_an_integer_type)
{
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    int16_t value = 0x1020;
    auto packetEnd = writeConfiguration<int64_t>(&buffer[0], 2, value);

    uint8_t expected[] = {PACKET_START_MARKER,
        PACKET_START_MARKER,
        'u',
        'P',
        12,
        2,
        0,
        0,
        0,
        0x20,
        0x10,
        0,
        0,
        0,
        0,
        0,
        0,
        0xA2,
        0x73};
    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd), ElementsAreArray(expected));
}

TEST_F(ProtocolTest, it_formats_a_configuration_write_for_a_string)
{
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    string value = "abcdefgh";
    auto packetEnd = writeConfiguration(&buffer[0], 2, value);

    uint8_t expected[] = {PACKET_START_MARKER,
        PACKET_START_MARKER,
        'u',
        'P',
        12,
        2,
        0,
        0,
        0,
        'a',
        'b',
        'c',
        'd',
        'e',
        'f',
        'g',
        'h',
        0x45,
        0x9f};
    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd), ElementsAreArray(expected));
}

TEST_F(ProtocolTest, it_pads_a_string_smaller_than_8_bytes_with_zeros)
{
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    string value = "abcde";
    auto packetEnd = writeConfiguration(&buffer[0], 2, value);

    uint8_t expected[] = {PACKET_START_MARKER,
        PACKET_START_MARKER,
        'u',
        'P',
        12,
        2,
        0,
        0,
        0,
        'a',
        'b',
        'c',
        'd',
        'e',
        0,
        0,
        0,
        0x13,
        0x47};

    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd), ElementsAreArray(expected));
}

TEST_F(ProtocolTest, it_throws_if_the_string_is_longer_than_8_bytes)
{
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    string value = "abcdefghijk";
    ASSERT_THROW(writeConfiguration(&buffer[0], 2, value), std::invalid_argument);
}

TEST_F(ProtocolTest, it_queries_a_single_parameter)
{
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    auto packetEnd = queryConfigurationParameter(&buffer[0], 2);

    uint8_t expected[] =
        {PACKET_START_MARKER, PACKET_START_MARKER, 'g', 'P', 4, 2, 0, 0, 0, 0xA6, 0xD6};

    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd), ElementsAreArray(expected));
}

TEST_F(ProtocolTest, it_parses_a_single_integer_parameter)
{
    std::vector<uint8_t> buffer = {2, 0, 0, 0, 3, 2, 1, 0, 0, 0, 0, 0};
    int64_t value = parseConfigurationParameter<int64_t>(&buffer[0], 12, 2);
    ASSERT_EQ(0x010203, value);
}

TEST_F(ProtocolTest, it_parses_a_single_string_parameter)
{
    std::vector<uint8_t> buffer = {2, 0, 0, 0, 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
    string value = parseConfigurationParameter<string>(&buffer[0], 12, 2);
    ASSERT_EQ("abcdefgh", value);
}

TEST_F(ProtocolTest, it_handles_zeroes_in_string_parameter_reads)
{
    std::vector<uint8_t> buffer = {2, 0, 0, 0, 'a', 'b', 0, 'd', 'e', 'f', 'g', 'h'};
    string value = parseConfigurationParameter<string>(&buffer[0], 12, 2);
    ASSERT_EQ("ab", value);
}

TEST_F(ProtocolTest, it_throws_if_the_buffer_is_bigger_than_12_bytes)
{
    ASSERT_THROW(
        {
            try {
                parseConfigurationParameter<string>(nullptr, 13, 2);
            }
            catch (std::invalid_argument const& e) {
                ASSERT_EQ(
                    "unexpected buffer size for gP response, expected 12 but got 13",
                    string(e.what()));
                throw;
            }
        },
        std::invalid_argument);
}

TEST_F(ProtocolTest, it_throws_if_the_buffer_is_smaller_than_12_bytes)
{
    ASSERT_THROW(
        {
            try {
                parseConfigurationParameter<string>(nullptr, 11, 2);
            }
            catch (std::invalid_argument const& e) {
                ASSERT_EQ(
                    "unexpected buffer size for gP response, expected 12 but got 11",
                    string(e.what()));
                throw;
            }
        },
        std::invalid_argument);
}

TEST_F(ProtocolTest, it_throws_if_the_parameter_is_not_the_expected_one)
{
    vector<uint8_t> buffer = {2, 0, 0, 0};
    ASSERT_THROW(
        {
            try {
                parseConfigurationParameter<string>(&buffer[0], 12, 3);
            }
            catch (std::invalid_argument const& e) {
                ASSERT_EQ("was expecting a read of configuration parameter 3 but got 2",
                    string(e.what()));
                throw;
            }
        },
        std::invalid_argument);
}

TEST_F(ProtocolTest, it_formats_a_firmware_block_write_message)
{
    vector<uint8_t> block = {1, 2, 3, 4};
    vector<uint8_t> packet(MAX_PACKET_SIZE, 0);

    auto packetEnd = queryAppBlockWrite(&packet[0], 0x10203040, &block[0], 4);

    uint8_t expected[] = {PACKET_START_MARKER,
        PACKET_START_MARKER,
        'W',
        'A',
        9,
        0x10,
        0x20,
        0x30,
        0x40,
        4,
        1,
        2,
        3,
        4,
        0x6d,
        0xad};
    ASSERT_THAT(std::vector<uint8_t>(&packet[0], packetEnd), ElementsAreArray(expected));
}

vector<uint8_t> make_byte_sequence(int size)
{
    vector<uint8_t> bytes;
    bytes.resize(size);
    for (int i = 0; i < size; ++i) {
        bytes[i] = i;
    }
    return bytes;
}

template<typename M>
static bool isMatrixValid(M const& m) {
    for (int i = 0; i < m.rows(); ++i) {
        for (int j = 0; j < m.cols(); ++j) {
            if (base::isUnknown(m(i, j))) {
                return false;
            }
        }
    }
    return true;
}

TEST_F(ProtocolTest, it_parses_a_status_message)
{
    vector<uint8_t> payload(make_byte_sequence(34));
    payload[33] = 0x2c;
    auto status = parseStatus(&payload[0], payload.size());
    ASSERT_EQ(0x03020100, status.time.toMilliseconds());
    ASSERT_EQ(0x07060504, status.extended_periodic_packets_overflow);
    ASSERT_EQ(0x0b0a0908, status.gps_updates);
    ASSERT_EQ(0x0f0e0d0c, status.last_gps_message.toMilliseconds());
    ASSERT_EQ(0x13121110, status.last_good_gps.toMilliseconds());
    ASSERT_EQ(0x17161514, status.last_usable_gps_velocity.toMilliseconds());
    ASSERT_EQ(0x1b1a1918, status.gps_rx);
    ASSERT_EQ(0x1d1c, status.gps_overflows);
    ASSERT_EQ(0x1f1e, round(status.hdop * 10));
    ASSERT_EQ(0x20, round(status.temperature.getCelsius()));

    ASSERT_EQ(4, status.filter_state.mode);
    ASSERT_EQ(0x5, status.filter_state.status);
}

TEST_F(ProtocolTest,
    it_throws_if_the_payload_buffer_is_smaller_than_the_expected_status_size)
{
    vector<uint8_t> payload;
    payload.resize(33);
    ASSERT_THROW_MESSAGE(parseStatus(&payload[0], payload.size()),
        std::invalid_argument,
        "buffer too small");
}

TEST_F(ProtocolTest,
    it_throws_if_the_payload_buffer_is_bigger_than_the_expected_status_size)
{
    vector<uint8_t> payload;
    payload.resize(35);
    ASSERT_THROW_MESSAGE(parseStatus(&payload[0], payload.size()),
        std::invalid_argument,
        "received status structure bigger than expected");
}

TEST_F(ProtocolTest, it_converts_ned_to_nwu)
{
    base::Orientation orientation;
    orientation = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    orientation = valueNED2NWU(orientation);
    base::Orientation expected;
    expected = Eigen::AngleAxisd(-M_PI / 4, Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    ASSERT_EQ(true, expected.isApprox(orientation));
}

TEST_F(ProtocolTest, it_parses_a_e4_INS_message)
{
    uint8_t msg[] = {
        0x10, 0x20, 0x30, 0x40, // time
        0x04, // INS
        0, 0, 0, 0, // [0, 1, 2, 3] -> q[4]
        0, 0, 128, 63,
        0, 0, 0, 64,
        0, 0, 64, 64,
        0, 0, 128, 64, // [4, 5, 6] -> angular velocity [3]
        0, 0, 160, 64,
        0, 0, 192, 64,
        0, 0, 224, 64, // [7, 8, 9] -> velocity[3]
        0, 0, 0, 65,
        0, 0, 16, 65,
        0, 0, 0, 0, 0, 0, 36, 64, // 10, latitude
        0, 0, 0, 0, 0, 0, 38, 64, // 11, longitude
        0, 0, 0, 0, 0, 0, 40, 64, // 12, altitude
        0, 0, 80, 65, // [13, 14, 15] -> mag[3]
        0, 0, 96, 65,
        0, 0, 112, 65,
        0, 0, 128, 65, // [16, 17, 18] -> measuredEulerAngles[3]
        0, 0, 136, 65,
        0, 0, 144, 65,
        0, 0, 152, 65 // 19 -> magneticDeclination
    };

    auto expected_time = base::Time::fromMilliseconds(0x40302010);
    auto update = parseE4Output(msg, sizeof(msg));

    constexpr double deg2rad = M_PI / 180;

    ASSERT_EQ(update.filter_state.time, expected_time);
    ASSERT_EQ(update.filter_state.mode, OPMODE_INS);
    ASSERT_EQ(update.filter_state.status, 0);

    ASSERT_EQ(update.rbs.time, expected_time);
    auto ned_q = Eigen::Quaterniond(0, 1, 2, 3);
    auto nwu_q = valueNED2NWU(ned_q);
    ASSERT_TRUE(update.rbs.orientation.isApprox(nwu_q));
    ASSERT_TRUE(update.rbs.angular_velocity.isApprox(Eigen::Vector3d(4, 5, 6) * deg2rad));
    auto ned_v = Eigen::Vector3d(7, 8, 9);
    auto nwu_v = ned_v.cwiseProduct(Eigen::Vector3d(1, -1, -1));
    ASSERT_TRUE(update.rbs.velocity.isApprox(nwu_v));
    ASSERT_TRUE(update.latitude.isApprox(base::Angle::fromDeg(10)));
    ASSERT_TRUE(update.longitude.isApprox(base::Angle::fromDeg(11)));
    ASSERT_TRUE(base::isUnknown(update.rbs.position.x()));
    ASSERT_TRUE(base::isUnknown(update.rbs.position.y()));
    ASSERT_NEAR(update.rbs.position.z(), 12, 1e-6);
    ASSERT_EQ(update.magnetic_info.time, expected_time);
    ASSERT_TRUE(update.magnetic_info.magnetometers.isApprox(Eigen::Vector3d(13, 14, 15)));
    ASSERT_TRUE(update.magnetic_info.measured_euler_angles.isApprox(Eigen::Vector3d(16, 17, 18)));
    ASSERT_NEAR(update.magnetic_info.declination.getRad(), base::Angle::fromRad(19).getRad(), 1e-6);

    ASSERT_TRUE(update.rba.time.isNull());
    ASSERT_FALSE(isMatrixValid(update.covQuaternion));
    ASSERT_TRUE(base::isUnknown(update.board_temperature.getCelsius()));
}

TEST_F(ProtocolTest, it_ignores_the_position_in_e4_if_the_mode_is_not_INS)
{
    uint8_t msg[] = {
        0x10, 0x20, 0x30, 0x40, // time
        0x02, // AHRS_HIGH_GAIN
        0, 0, 0, 0, // [0, 1, 2, 3] -> q[4]
        0, 0, 128, 63,
        0, 0, 0, 64,
        0, 0, 64, 64,
        0, 0, 128, 64, // [4, 5, 6] -> angular velocity [3]
        0, 0, 160, 64,
        0, 0, 192, 64,
        0, 0, 224, 64, // [7, 8, 9] -> velocity[3]
        0, 0, 0, 65,
        0, 0, 16, 65,
        0, 0, 0, 0, 0, 0, 36, 64, // 10, latitude
        0, 0, 0, 0, 0, 0, 38, 64, // 11, longitude
        0, 0, 0, 0, 0, 0, 40, 64, // 12, altitude
        0, 0, 80, 65, // [13, 14, 15] -> mag[3]
        0, 0, 96, 65,
        0, 0, 112, 65,
        0, 0, 128, 65, // [16, 17, 18] -> measuredEulerAngles[3]
        0, 0, 136, 65,
        0, 0, 144, 65,
        0, 0, 152, 65 // 19 -> magneticDeclination
    };

    auto expected_time = base::Time::fromMilliseconds(0x40302010);
    auto update = parseE4Output(msg, sizeof(msg));

    constexpr double deg2rad = M_PI / 180;

    ASSERT_EQ(update.filter_state.time, expected_time);
    ASSERT_EQ(update.filter_state.mode, OPMODE_AHRS_HIGH_GAIN);
    ASSERT_EQ(update.filter_state.status, 0);

    ASSERT_EQ(update.rbs.time, expected_time);
    auto ned_q = Eigen::Quaterniond(0, 1, 2, 3);
    auto nwu_q = valueNED2NWU(ned_q);
    ASSERT_TRUE(update.rbs.orientation.isApprox(nwu_q));
    ASSERT_TRUE(update.rbs.angular_velocity.isApprox(Eigen::Vector3d(4, 5, 6) * deg2rad));
    ASSERT_FALSE(update.rbs.hasValidVelocity());
    ASSERT_TRUE(base::isUnknown(update.latitude));
    ASSERT_TRUE(base::isUnknown(update.longitude));
    ASSERT_FALSE(update.rbs.hasValidPosition());
    ASSERT_EQ(update.magnetic_info.time, expected_time);
    ASSERT_TRUE(update.magnetic_info.magnetometers.isApprox(Eigen::Vector3d(13, 14, 15)));
    ASSERT_TRUE(update.magnetic_info.measured_euler_angles.isApprox(Eigen::Vector3d(16, 17, 18)));
    ASSERT_NEAR(update.magnetic_info.declination.getRad(), base::Angle::fromRad(19).getRad(), 1e-6);

    ASSERT_TRUE(update.rba.time.isNull());
    ASSERT_FALSE(isMatrixValid(update.covQuaternion));
    ASSERT_TRUE(base::isUnknown(update.board_temperature.getCelsius()));
}

TEST_F(ProtocolTest, it_outputs_no_pose_related_data_if_the_mode_is_below_AHRS)
{
    uint8_t msg[] = {
        0x10, 0x20, 0x30, 0x40, // time
        0x01,
        0, 0, 0, 0, // [0, 1, 2, 3] -> q[4]
        0, 0, 128, 63,
        0, 0, 0, 64,
        0, 0, 64, 64,
        0, 0, 128, 64, // [4, 5, 6] -> angular velocity [3]
        0, 0, 160, 64,
        0, 0, 192, 64,
        0, 0, 224, 64, // [7, 8, 9] -> velocity[3]
        0, 0, 0, 65,
        0, 0, 16, 65,
        0, 0, 0, 0, 0, 0, 36, 64, // 10, latitude
        0, 0, 0, 0, 0, 0, 38, 64, // 11, longitude
        0, 0, 0, 0, 0, 0, 40, 64, // 12, altitude
        0, 0, 80, 65, // [13, 14, 15] -> mag[3]
        0, 0, 96, 65,
        0, 0, 112, 65,
        0, 0, 128, 65, // [16, 17, 18] -> measuredEulerAngles[3]
        0, 0, 136, 65,
        0, 0, 144, 65,
        0, 0, 152, 65 // 19 -> magneticDeclination
    };

    auto expected_time = base::Time::fromMilliseconds(0x40302010);
    auto update = parseE4Output(msg, sizeof(msg));

    ASSERT_EQ(update.filter_state.time, expected_time);
    ASSERT_EQ(update.filter_state.mode, OPMODE_INITIALIZING);
    ASSERT_EQ(update.filter_state.status, 0);

    ASSERT_TRUE(update.rbs.time.isNull());
    ASSERT_TRUE(update.magnetic_info.time.isNull());
    ASSERT_TRUE(update.rba.time.isNull());
    ASSERT_FALSE(isMatrixValid(update.covQuaternion));
    ASSERT_TRUE(base::isUnknown(update.board_temperature.getCelsius()));
}

TEST_F(ProtocolTest, it_parses_a_e5_INS_message)
{
    uint8_t msg[] = {
        0x10, 0x20, 0x30, 0x40, // time
        0x04, // INS
        0xe2, // -30 -> temperature
        0, 0, 0, 0, // [0, 1, 2, 3] -> q[4]
        0, 0, 128, 63,
        0, 0, 0, 64,
        0, 0, 64, 64,
        0, 0, 128, 64, // [4, 5, 6] -> angular velocity [3]
        0, 0, 160, 64,
        0, 0, 192, 64,
        0, 0, 224, 64, // [7, 8, 9] -> velocity[3]
        0, 0, 0, 65,
        0, 0, 16, 65,
        0, 0, 0, 0, 0, 0, 36, 64, // 10, latitude
        0, 0, 0, 0, 0, 0, 38, 64, // 11, longitude
        0, 0, 64, 65, // 12, altitude
        0, 0, 80, 65, // [13, 14, 15] -> mag[3]
        0, 0, 96, 65,
        0, 0, 112, 65,
        0, 0, 128, 65, // [16, 17, 18] -> measuredEulerAngles[3]
        0, 0, 136, 65,
        0, 0, 144, 65,
        0, 0, 152, 65, // 19 -> magneticDeclination
        0, 0, 160, 65, // [20, 21, 22] -> accelerations[3]
        0, 0, 168, 65,
        0, 0, 176, 65,
        0, 0, 184, 65, // [23, 24, 25, 26, 27, 28] -> covPosition[6]
        0, 0, 192, 65,
        0, 0, 200, 65,
        0, 0, 208, 65,
        0, 0, 216, 65,
        0, 0, 224, 65,
        0, 0, 232, 65, // [29, 30, 31, 32, 33, 34] -> covVelocity[6]
        0, 0, 240, 65,
        0, 0, 248, 65,
        0, 0, 0, 66,
        0, 0, 4, 66,
        0, 0, 8, 66,
        0, 0, 12, 66, // [35, 36, 37, 38, 39, 40, 41, 42, 43, 44] -> covQuaternion[10]
        0, 0, 16, 66,
        0, 0, 20, 66,
        0, 0, 24, 66,
        0, 0, 28, 66,
        0, 0, 32, 66,
        0, 0, 36, 66,
        0, 0, 40, 66,
        0, 0, 44, 66,
        0, 0, 48, 66
    };

    auto expected_time = base::Time::fromMilliseconds(0x40302010);
    auto update = parseE5Output(msg, sizeof(msg));

    ASSERT_EQ(update.filter_state.time, expected_time);
    ASSERT_EQ(update.filter_state.mode, OPMODE_INS);
    ASSERT_EQ(update.filter_state.status, 0);

    ASSERT_EQ(update.rbs.time, expected_time);
    auto ned_q = Eigen::Quaterniond(0, 1, 2, 3);
    auto nwu_q = valueNED2NWU(ned_q);
    ASSERT_TRUE(update.rbs.orientation.isApprox(nwu_q));
    ASSERT_TRUE(update.rbs.angular_velocity.isApprox(Eigen::Vector3d(4, 5, 6)));
    auto ned_v = Eigen::Vector3d(7, 8, 9);
    auto nwu_v = ned_v.cwiseProduct(Eigen::Vector3d(1, -1, -1));
    ASSERT_TRUE(update.rbs.velocity.isApprox(nwu_v));
    ASSERT_NEAR(update.latitude.getRad(), base::Angle::fromRad(10).getRad(), 1e-6);
    ASSERT_NEAR(update.longitude.getRad(), base::Angle::fromRad(11).getRad(), 1e-6);
    ASSERT_TRUE(base::isUnknown(update.rbs.position.x()));
    ASSERT_TRUE(base::isUnknown(update.rbs.position.y()));
    ASSERT_NEAR(update.rbs.position.z(), 12, 1e-6);
    ASSERT_EQ(update.magnetic_info.time, expected_time);
    ASSERT_TRUE(update.magnetic_info.magnetometers.isApprox(Eigen::Vector3d(13, 14, 15)));
    ASSERT_TRUE(update.magnetic_info.measured_euler_angles.isApprox(Eigen::Vector3d(16, 17, 18)));
    ASSERT_NEAR(update.magnetic_info.declination.getRad(), base::Angle::fromRad(19).getRad(), 1e-6);
    ASSERT_EQ(update.rba.time, expected_time);
    ASSERT_TRUE(update.rba.acceleration.isApprox(Eigen::Vector3d(20, 21, 22)));

    Eigen::Matrix3d expectedCovPosition;
    expectedCovPosition
        << 23.0, 24.0, 25.0,
           24.0, 26.0, 27.0,
           25.0, 27.0, 28.0;
    ASSERT_TRUE(update.rbs.cov_position.isApprox(expectedCovPosition));

    Eigen::Matrix3d expectedCovVelocity;
    expectedCovVelocity
        << 29.0, 30.0, 31.0,
           30.0, 32.0, 33.0,
           31.0, 33.0, 34.0;
    ASSERT_TRUE(update.rbs.cov_velocity.isApprox(expectedCovVelocity));

    Eigen::Matrix4d expectedCovQuaternion;
    expectedCovQuaternion
        << 35.0, 36.0, 37.0, 38.0,
           36.0, 39.0, 40.0, 41.0,
           37.0, 40.0, 42.0, 43.0,
           38.0, 41.0, 43.0, 44.0;
    ASSERT_TRUE(update.covQuaternion.isApprox(expectedCovQuaternion));
    ASSERT_EQ(update.board_temperature.getCelsius(), -30);
}

TEST_F(ProtocolTest, it_ignores_the_position_in_e5_if_the_mode_is_not_INS)
{
    uint8_t msg[] = {
        0x10, 0x20, 0x30, 0x40, // time
        0x02, // AHRS High Gain
        0xe2, // -30 -> temperature
        0, 0, 0, 0, // [0, 1, 2, 3] -> q[4]
        0, 0, 128, 63,
        0, 0, 0, 64,
        0, 0, 64, 64,
        0, 0, 128, 64, // [4, 5, 6] -> angular velocity [3]
        0, 0, 160, 64,
        0, 0, 192, 64,
        0, 0, 224, 64, // [7, 8, 9] -> velocity[3]
        0, 0, 0, 65,
        0, 0, 16, 65,
        0, 0, 0, 0, 0, 0, 36, 64, // 10, latitude
        0, 0, 0, 0, 0, 0, 38, 64, // 11, longitude
        0, 0, 64, 65, // 12, altitude
        0, 0, 80, 65, // [13, 14, 15] -> mag[3]
        0, 0, 96, 65,
        0, 0, 112, 65,
        0, 0, 128, 65, // [16, 17, 18] -> measuredEulerAngles[3]
        0, 0, 136, 65,
        0, 0, 144, 65,
        0, 0, 152, 65, // 19 -> magneticDeclination
        0, 0, 160, 65, // [20, 21, 22] -> accelerations[3]
        0, 0, 168, 65,
        0, 0, 176, 65,
        0, 0, 184, 65, // [23, 24, 25, 26, 27, 28] -> covPosition[6]
        0, 0, 192, 65,
        0, 0, 200, 65,
        0, 0, 208, 65,
        0, 0, 216, 65,
        0, 0, 224, 65,
        0, 0, 232, 65, // [29, 30, 31, 32, 33, 34] -> covVelocity[6]
        0, 0, 240, 65,
        0, 0, 248, 65,
        0, 0, 0, 66,
        0, 0, 4, 66,
        0, 0, 8, 66,
        0, 0, 12, 66, // [35, 36, 37, 38, 39, 40, 41, 42, 43, 44] -> covQuaternion[10]
        0, 0, 16, 66,
        0, 0, 20, 66,
        0, 0, 24, 66,
        0, 0, 28, 66,
        0, 0, 32, 66,
        0, 0, 36, 66,
        0, 0, 40, 66,
        0, 0, 44, 66,
        0, 0, 48, 66
    };

    auto expected_time = base::Time::fromMilliseconds(0x40302010);
    auto update = parseE5Output(msg, sizeof(msg));

    ASSERT_EQ(update.filter_state.time, expected_time);
    ASSERT_EQ(update.filter_state.mode, OPMODE_AHRS_HIGH_GAIN);
    ASSERT_EQ(update.filter_state.status, 0);

    ASSERT_EQ(update.rbs.time, expected_time);
    auto ned_q = Eigen::Quaterniond(0, 1, 2, 3);
    auto nwu_q = valueNED2NWU(ned_q);
    ASSERT_TRUE(update.rbs.orientation.isApprox(nwu_q));
    ASSERT_TRUE(update.rbs.angular_velocity.isApprox(Eigen::Vector3d(4, 5, 6)));
    ASSERT_FALSE(update.rbs.hasValidVelocity());
    ASSERT_TRUE(base::isUnknown(update.latitude.getRad()));
    ASSERT_TRUE(base::isUnknown(update.longitude.getRad()));
    ASSERT_FALSE(update.rbs.hasValidPosition());

    ASSERT_EQ(update.magnetic_info.time, expected_time);
    ASSERT_TRUE(update.magnetic_info.magnetometers.isApprox(Eigen::Vector3d(13, 14, 15)));
    ASSERT_TRUE(update.magnetic_info.measured_euler_angles.isApprox(Eigen::Vector3d(16, 17, 18)));
    ASSERT_NEAR(update.magnetic_info.declination.getRad(), base::Angle::fromRad(19).getRad(), 1e-6);
    ASSERT_EQ(update.rba.time, expected_time);
    ASSERT_TRUE(update.rba.acceleration.isApprox(Eigen::Vector3d(20, 21, 22)));

    ASSERT_FALSE(update.rbs.hasValidPositionCovariance());
    ASSERT_FALSE(update.rbs.hasValidVelocityCovariance());

    Eigen::Matrix4d expectedCovQuaternion;
    expectedCovQuaternion
        << 35.0, 36.0, 37.0, 38.0,
           36.0, 39.0, 40.0, 41.0,
           37.0, 40.0, 42.0, 43.0,
           38.0, 41.0, 43.0, 44.0;
    ASSERT_TRUE(update.covQuaternion.isApprox(expectedCovQuaternion));
    ASSERT_EQ(update.board_temperature.getCelsius(), -30);
}

TEST_F(ProtocolTest, it_returns_invalid_data_if_the_state_is_below_AHRS)
{
    uint8_t msg[] = {
        0x10, 0x20, 0x30, 0x40, // time
        0x01, // AHRS High Gain
        0xe2, // -30 -> temperature
        0, 0, 0, 0, // [0, 1, 2, 3] -> q[4]
        0, 0, 128, 63,
        0, 0, 0, 64,
        0, 0, 64, 64,
        0, 0, 128, 64, // [4, 5, 6] -> angular velocity [3]
        0, 0, 160, 64,
        0, 0, 192, 64,
        0, 0, 224, 64, // [7, 8, 9] -> velocity[3]
        0, 0, 0, 65,
        0, 0, 16, 65,
        0, 0, 0, 0, 0, 0, 36, 64, // 10, latitude
        0, 0, 0, 0, 0, 0, 38, 64, // 11, longitude
        0, 0, 64, 65, // 12, altitude
        0, 0, 80, 65, // [13, 14, 15] -> mag[3]
        0, 0, 96, 65,
        0, 0, 112, 65,
        0, 0, 128, 65, // [16, 17, 18] -> measuredEulerAngles[3]
        0, 0, 136, 65,
        0, 0, 144, 65,
        0, 0, 152, 65, // 19 -> magneticDeclination
        0, 0, 160, 65, // [20, 21, 22] -> accelerations[3]
        0, 0, 168, 65,
        0, 0, 176, 65,
        0, 0, 184, 65, // [23, 24, 25, 26, 27, 28] -> covPosition[6]
        0, 0, 192, 65,
        0, 0, 200, 65,
        0, 0, 208, 65,
        0, 0, 216, 65,
        0, 0, 224, 65,
        0, 0, 232, 65, // [29, 30, 31, 32, 33, 34] -> covVelocity[6]
        0, 0, 240, 65,
        0, 0, 248, 65,
        0, 0, 0, 66,
        0, 0, 4, 66,
        0, 0, 8, 66,
        0, 0, 12, 66, // [35, 36, 37, 38, 39, 40, 41, 42, 43, 44] -> covQuaternion[10]
        0, 0, 16, 66,
        0, 0, 20, 66,
        0, 0, 24, 66,
        0, 0, 28, 66,
        0, 0, 32, 66,
        0, 0, 36, 66,
        0, 0, 40, 66,
        0, 0, 44, 66,
        0, 0, 48, 66
    };

    auto expected_time = base::Time::fromMilliseconds(0x40302010);
    auto update = parseE5Output(msg, sizeof(msg));

    ASSERT_EQ(update.filter_state.time, expected_time);
    ASSERT_EQ(update.filter_state.mode, OPMODE_INITIALIZING);
    ASSERT_EQ(update.filter_state.status, 0);

    ASSERT_TRUE(update.rbs.time.isNull());
    ASSERT_TRUE(base::isUnknown(update.latitude.getRad()));
    ASSERT_TRUE(base::isUnknown(update.longitude.getRad()));
    ASSERT_TRUE(update.magnetic_info.time.isNull());
    ASSERT_TRUE(update.rba.time.isNull());

    ASSERT_FALSE(isMatrixValid(update.covQuaternion));
    ASSERT_EQ(update.board_temperature.getCelsius(), -30);
}