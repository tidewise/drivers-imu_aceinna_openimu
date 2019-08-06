#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <imu_aceinna_openimu/Protocol.hpp>

using namespace std;
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

TEST_F(ProtocolTest, it_formats_a_configuration_write_for_an_integer_type) {
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    int16_t value = 0x1020;
    auto packetEnd = writeConfiguration<int64_t>(&buffer[0], 2, value);

    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd),
        ElementsAre(PACKET_START_MARKER, PACKET_START_MARKER, 'u', 'P', 12,
                    2, 0, 0, 0,
                    0x20, 0x10, 0, 0, 0, 0, 0, 0,
                    0xA2, 0x73));
}

TEST_F(ProtocolTest, it_formats_a_configuration_write_for_a_string) {
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    string value = "abcdefgh";
    auto packetEnd = writeConfiguration(&buffer[0], 2, value);

    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd),
        ElementsAre(PACKET_START_MARKER, PACKET_START_MARKER, 'u', 'P', 12,
                    2, 0, 0, 0,
                    'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h',
                    0x45, 0x9f));
}

TEST_F(ProtocolTest, it_pads_a_string_smaller_than_8_bytes_with_zeros) {
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    string value = "abcde";
    auto packetEnd = writeConfiguration(&buffer[0], 2, value);

    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd),
        ElementsAre(PACKET_START_MARKER, PACKET_START_MARKER, 'u', 'P', 12,
                    2, 0, 0, 0,
                    'a', 'b', 'c', 'd', 'e', 0, 0, 0,
                    0x13, 0x47));
}

TEST_F(ProtocolTest, it_throws_if_the_string_is_longer_than_8_bytes) {
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    string value = "abcdefghijk";
    ASSERT_THROW(writeConfiguration(&buffer[0], 2, value), std::invalid_argument);
}

TEST_F(ProtocolTest, it_queries_a_single_parameter) {
    std::vector<uint8_t> buffer(MAX_PACKET_SIZE, 0);
    auto packetEnd = queryConfigurationParameter(&buffer[0], 2);

    ASSERT_THAT(std::vector<uint8_t>(&buffer[0], packetEnd),
        ElementsAre(PACKET_START_MARKER, PACKET_START_MARKER, 'g', 'P', 4,
                    2, 0, 0, 0, 0xA6, 0xD6));
}

TEST_F(ProtocolTest, it_parses_a_single_integer_parameter) {
    std::vector<uint8_t> buffer = {
        2, 0, 0, 0,
        3, 2, 1, 0, 0, 0, 0, 0
    };
    int64_t value = parseConfigurationParameter<int64_t>(&buffer[0], 12, 2);
    ASSERT_EQ(0x010203, value);
}

TEST_F(ProtocolTest, it_parses_a_single_string_parameter) {
    std::vector<uint8_t> buffer = {
        2, 0, 0, 0,
        'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'
    };
    string value = parseConfigurationParameter<string>(&buffer[0], 12, 2);
    ASSERT_EQ("abcdefgh", value);
}

TEST_F(ProtocolTest, it_throws_if_the_buffer_is_bigger_than_12_bytes) {
    ASSERT_THROW({
        try {
            parseConfigurationParameter<string>(nullptr, 13, 2);
        }
        catch(std::invalid_argument const& e) {
            ASSERT_EQ("unexpected buffer size for gP response, expected 12 but got 13",
                      string(e.what()));
            throw;
        }
    }, std::invalid_argument);
}

TEST_F(ProtocolTest, it_throws_if_the_buffer_is_smaller_than_12_bytes) {
    ASSERT_THROW({
        try {
            parseConfigurationParameter<string>(nullptr, 11, 2);
        }
        catch(std::invalid_argument const& e) {
            ASSERT_EQ("unexpected buffer size for gP response, expected 12 but got 11",
                      string(e.what()));
            throw;
        }
    }, std::invalid_argument);
}

TEST_F(ProtocolTest, it_throws_if_the_parameter_is_not_the_expected_one) {
    vector<uint8_t> buffer = { 2, 0, 0, 0 };
    ASSERT_THROW({
        try {
            parseConfigurationParameter<string>(&buffer[0], 12, 3);
        }
        catch(std::invalid_argument const& e) {
            ASSERT_EQ("was expecting a read of configuration parameter 3 but got 2",
                      string(e.what()));
            throw;
        }
    }, std::invalid_argument);
}