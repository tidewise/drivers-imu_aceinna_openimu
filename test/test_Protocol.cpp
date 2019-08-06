#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <imu_aceinna_openimu/Protocol.hpp>

using namespace imu_aceinna_openimu::protocol;
using testing::ElementsAre;

struct ProtocolTest : public ::testing::Test {
    std::vector<uint8_t> buffer;

    ProtocolTest() {
        buffer.resize(256, 0);
    }
};

TEST_F(ProtocolTest, it_waits_for_more_bytes_if_the_buffer_is_too_small) {
    ASSERT_EQ(0, extractPacket(&buffer[0], 4));
}
TEST_F(ProtocolTest, it_finds_a_packet_start_within_the_packet) {
    buffer[5] = PACKET_START_MARKER;
    buffer[6] = PACKET_START_MARKER;
    ASSERT_EQ(-5, extractPacket(&buffer[0], 9));
}
TEST_F(ProtocolTest, it_handles_a_start_byte_at_the_very_end_of_the_buffer) {
    buffer[9] = PACKET_START_MARKER;
    ASSERT_EQ(-9, extractPacket(&buffer[0], 10));
}
TEST_F(ProtocolTest, it_rejects_a_packet_with_an_invalid_CRC) {
    buffer[0] = PACKET_START_MARKER;
    buffer[1] = PACKET_START_MARKER;
    ASSERT_EQ(-1, extractPacket(&buffer[0], 10));
}
TEST_F(ProtocolTest, it_accepts_a_packet_that_starts_at_zero_and_has_a_valid_CRC) {
    buffer = { PACKET_START_MARKER, PACKET_START_MARKER, 'p', 'G', 0, 0x5d, 0x5f };
    ASSERT_EQ(7, extractPacket(&buffer[0], 10));
}

TEST_F(ProtocolTest, it_formats_a_device_info_query) {
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    auto packetEnd = queryDeviceInfo(&buffer[0]);

    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd),
        ElementsAre(PACKET_START_MARKER, PACKET_START_MARKER, 'p', 'G', 0, 0x5d, 0x5f));
}

TEST_F(ProtocolTest, it_parses_a_device_info_response) {
    std::vector<uint8_t> buffer = { 'a', 'b', 'C', 'd' };

    ASSERT_EQ("abCd", parseDeviceInfo(&buffer[0], 4));
}

TEST_F(ProtocolTest, it_formats_a_configuration_query) {
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    auto packetEnd = queryConfiguration(&buffer[0]);

    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd),
        ElementsAre(PACKET_START_MARKER, PACKET_START_MARKER, 'g', 'A', 0, 0x31, 0x0A));
}

TEST_F(ProtocolTest, it_parses_a_configuration_response) {
    std::vector<uint8_t> buffer = {
        1, 0, 0, 0, 0, 0, 0, 0, // Data CRC
        2, 0, 0, 0, 0, 0, 0, 0, // Data Size
        3, 0, 0, 0, 0, 0, 0, 0, // Baud Rate
        'g', 'A', ' ', ' ', ' ', ' ', ' ', ' ', // Periodic Packet Type
        4, 0, 0, 0, 0, 0, 0, 0, // Periodic Packet Rate
        5, 0, 0, 0, 0, 0, 0, 0, // Accel low-pass filter
        6, 0, 0, 0, 0, 0, 0, 0, // Rate low-pass filter
        '+', 'X', '+', 'Y', '+', 'Z', ' ', ' ' // Orientation
    };

    auto conf = parseConfiguration(&buffer[0], 64);
    ASSERT_EQ("gA", conf.periodic_packet_type);
    ASSERT_EQ(4, conf.periodic_packet_rate);
    ASSERT_EQ(5, conf.acceleration_low_pass_filter);
    ASSERT_EQ(6, conf.angular_velocity_low_pass_filter);
    ASSERT_EQ("+X+Y+Z  ", conf.orientation);
}

TEST_F(ProtocolTest, it_raises_if_configuration_buffer_is_smaller_than_the_expected_struct) {
    ASSERT_THROW(parseConfiguration(nullptr, 55), std::invalid_argument);
}

TEST_F(ProtocolTest, it_limits_the_orientation_field_to_the_buffer_size_if_smaller_than_64_bytes) {
    std::vector<uint8_t> buffer;
    buffer.resize(56, 0);
    buffer.insert(buffer.end(), { 'a', 'b', 'c', 'd' });

    auto conf = parseConfiguration(&buffer[0], 60);
    ASSERT_EQ("abcd", conf.orientation);
}

TEST_F(ProtocolTest, it_limits_the_orientation_field_size_to_8_bytes_regardless_of_the_buffer_size) {
    std::vector<uint8_t> buffer;
    buffer.resize(56, 0);
    buffer.insert(buffer.end(), { 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i' });

    auto conf = parseConfiguration(&buffer[0], 65);
    ASSERT_EQ("abcdefgh", conf.orientation);
}