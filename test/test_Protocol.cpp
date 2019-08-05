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
