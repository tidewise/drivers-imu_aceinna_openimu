#include <gtest/gtest.h>
#include <imu_aceinna_openimu/Driver.hpp>
#include <imu_aceinna_openimu/Protocol.hpp>
#include <iodrivers_base/FixtureGTest.hpp>

using namespace std;
using namespace imu_aceinna_openimu;

#define ASSERT_THROW_MESSAGE(code, exception, message) \
    ASSERT_THROW({ \
        try { code; } \
        catch(exception& e) { \
            ASSERT_EQ(message, string(e.what())); \
            throw; \
        } \
    }, exception)

struct DriverTest : public ::testing::Test, iodrivers_base::Fixture<Driver> {
    void openDriver() {
        driver.openURI("test://");
    }
};

TEST_F(DriverTest, it_queries_the_device_description_on_open) {
    driver.openURI("test://");

    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'p', 'G', 0, 0x5d, 0x5f },
        vector<uint8_t>{ 0x55, 0x55, 'p', 'G', 4, 'a', 'b', 'c', 'd', 0x1f, 0x72 });
    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'g', 'V', 0, 0xab, 0xee },
        vector<uint8_t>{ 0x55, 0x55, 'g', 'V', 5, 'I', 'N', 'S', 'T', 'W', 0x39, 0x0f });
    auto info = driver.readDeviceInfo();
    ASSERT_EQ("abcd", info.device_id);
    ASSERT_EQ("INSTW", info.app_version);
}

TEST_F(DriverTest, it_raises_if_the_parameter_write_result_is_not_a_4_byte_value) {
    IODRIVERS_BASE_MOCK();
    openDriver();

    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'u', 'P', 12, 2, 0, 0, 0,
                         0x20, 0x10, 0, 0, 0, 0, 0, 0, 0xA2, 0x73 },
        vector<uint8_t>{ 0x55, 0x55, 'u', 'P', 3, 0xff, 0xff, 0xff, 0x6b, 0x21 });
    ASSERT_THROW_MESSAGE(
        driver.writeConfiguration<int64_t>(2, 0x1020, true),
        std::invalid_argument,
        "unexpected reply size for uP (write configuration parameter), "
        "expected 4 bytes, got 3");
}

TEST_F(DriverTest, it_raises_if_the_parameter_write_result_is_INVALID_PARAM) {
    IODRIVERS_BASE_MOCK();
    openDriver();

    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'u', 'P', 12, 2, 0, 0, 0,
                         0x20, 0x10, 0, 0, 0, 0, 0, 0, 0xA2, 0x73 },
        vector<uint8_t>{ 0x55, 0x55, 'u', 'P', 4, 0xff, 0xff, 0xff, 0xff, 0x85, 0xe9 });
    ASSERT_THROW_MESSAGE(
        driver.writeConfiguration<int64_t>(2, 0x1020, true),
        ConfigurationWriteFailed,
        "writing configuration parameter 2 failed: invalid parameter index");
}

TEST_F(DriverTest, it_raises_if_the_parameter_write_result_is_INVALID_VALUE) {
    IODRIVERS_BASE_MOCK();
    openDriver();

    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'u', 'P', 12, 2, 0, 0, 0,
                         0x20, 0x10, 0, 0, 0, 0, 0, 0, 0xA2, 0x73 },
        vector<uint8_t>{ 0x55, 0x55, 'u', 'P', 4, 0xfe, 0xff, 0xff, 0xff, 0xf3, 0x5d });
    ASSERT_THROW_MESSAGE(
        driver.writeConfiguration<int64_t>(2, 0x1020, true),
        ConfigurationWriteFailed,
        "writing configuration parameter 2 failed: invalid value 4128");
}

TEST_F(DriverTest, it_raises_if_the_parameter_write_result_is_an_unknown_failure) {
    IODRIVERS_BASE_MOCK();
    openDriver();

    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'u', 'P', 12, 2, 0, 0, 0,
                         0x20, 0x10, 0, 0, 0, 0, 0, 0, 0xA2, 0x73 },
        vector<uint8_t>{ 0x55, 0x55, 'u', 'P', 4, 0xfe, 0xff, 0x10, 0xff, 0xf3, 0xd1 });
    ASSERT_THROW_MESSAGE(
        driver.writeConfiguration<int64_t>(2, 0x1020, true),
        ConfigurationWriteFailed,
        "writing configuration parameter 2 failed: unknown error");
}

TEST_F(DriverTest, it_raises_if_the_property_actual_value_differs_from_the_written_if_validate_is_true) {
    IODRIVERS_BASE_MOCK();
    openDriver();

    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'u', 'P', 12, 2, 0, 0, 0,
                         0x20, 0x10, 0, 0, 0, 0, 0, 0, 0xA2, 0x73 },
        vector<uint8_t>{ 0x55, 0x55, 'u', 'P', 4, 0, 0, 0, 0, 0x1c, 0x26 });
    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'g', 'P', 4, 2, 0, 0, 0, 0xa6, 0xd6 },
        vector<uint8_t>{ 0x55, 0x55, 'g', 'P', 12, 2, 0, 0, 0,
                         0x20, 0x11, 0, 0, 0, 0, 0, 0, 0x19, 0x41 });
    ASSERT_THROW_MESSAGE(
        driver.writeConfiguration<int64_t>(2, 0x1020, true),
        ConfigurationWriteFailed,
        "writing configuration parameter 2 failed. "
        "Current property value is 4384, expected 4128");
}

TEST_F(DriverTest, it_processes_a_periodic_i1_message) {
    vector<uint8_t> i1 { 0x55, 0x55, 'i', '1', 34,
                         0x00, 0x01, 0x02, 0x03 };
    i1.resize(34 + 7, 0);
    i1[34 + 5] = 0xee;
    i1[34 + 6] = 0x85;

    pushDataToDriver(&i1[0], &i1[i1.size()]);
    openDriver();
    auto update = driver.processOne();

    ASSERT_EQ(Driver::UPDATED_STATUS, update.updated);
    ASSERT_TRUE(update.isUpdated(Driver::UPDATED_STATUS));
    ASSERT_EQ(0x03020100, driver.getIMUStatus().time.toMilliseconds());
}

TEST_F(DriverTest, it_processes_a_periodic_e3_message) {
    vector<uint8_t> e3 { 0x55, 0x55, 'e', '3', 137,
                         0x00, 0x01, 0x02, 0x03 };
    e3.resize(137 + 7, 0);
    e3[137 + 5] = 0xc1;
    e3[137 + 6] = 0xac;

    pushDataToDriver(&e3[0], &e3[e3.size()]);
    openDriver();
    auto update = driver.processOne();

    ASSERT_EQ(Driver::UPDATED_STATE, update.updated);
    ASSERT_TRUE(update.isUpdated(Driver::UPDATED_STATE));
    ASSERT_EQ(0x03020100, driver.getState().rbs.time.toMilliseconds());
}

TEST_F(DriverTest, it_demultiplexes_an_EP_message) {
    vector<uint8_t> i1 { 'i', '1', 34, 0x00, 0x01, 0x02, 0x03 };
    i1.resize(34 + 3, 0);
    vector<uint8_t> e3 { 'e', '3', 137, 0x04, 0x05, 0x06, 0x07 };
    e3.resize(137 + 3, 0);

    vector<uint8_t> message { 0x55, 0x55, 'E', 'P', 177 };
    message.insert(message.end(), i1.begin(), i1.end());
    message.insert(message.end(), e3.begin(), e3.end());
    message.resize(177 + 7, 0);
    message[177 + 5] = 0xa0;
    message[177 + 6] = 0x32;

    pushDataToDriver(&message[0], &message[message.size()]);
    openDriver();
    auto update = driver.processOne();

    ASSERT_EQ(Driver::UPDATED_STATE | Driver::UPDATED_STATUS, update.updated);
    ASSERT_TRUE(update.isUpdated(Driver::UPDATED_STATUS));
    ASSERT_TRUE(update.isUpdated(Driver::UPDATED_STATE));
    ASSERT_EQ(0x03020100, driver.getIMUStatus().time.toMilliseconds());
    ASSERT_EQ(0x07060504, driver.getState().rbs.time.toMilliseconds());
}

TEST_F(DriverTest, it_handles_payload_being_too_small_for_a_embedded_packet_header) {
    vector<uint8_t> i1 { 'i', '1', 34, 0x00, 0x01, 0x02, 0x03 };
    i1.resize(34 + 3, 0);
    vector<uint8_t> e3 { 'e', '3', 137, 0x04, 0x05, 0x06, 0x07 };
    e3.resize(137 + 3, 0);

    vector<uint8_t> message { 0x55, 0x55, 'E', 'P', 167 };
    message.insert(message.end(), i1.begin(), i1.end());
    message.insert(message.end(), e3.begin(), e3.end());
    message.resize(167 + 7, 0);
    message[167 + 5] = 0xcb;
    message[167 + 6] = 0xb5;

    pushDataToDriver(&message[0], &message[message.size()]);
    openDriver();
    ASSERT_THROW_MESSAGE(
        driver.processOne(),
        std::invalid_argument,
        "EP: payload too small for embedded message");
}

TEST_F(DriverTest, it_skips_EP_embedded_messages_whose_size_is_bigger_than_the_payload_size) {
    vector<uint8_t> i1 { 'i', '1', 34, 0x00, 0x01, 0x02, 0x03 };
    i1.resize(34 + 3, 0);
    vector<uint8_t> e3 { 'e', '3', 138, 0x04, 0x05, 0x06, 0x07 };
    e3.resize(137 + 3, 0);

    vector<uint8_t> message { 0x55, 0x55, 'E', 'P', 177 };
    message.insert(message.end(), i1.begin(), i1.end());
    message.insert(message.end(), e3.begin(), e3.end());
    message.resize(177 + 7, 0);
    message[177 + 5] = 0xfe;
    message[177 + 6] = 0xca;

    pushDataToDriver(&message[0], &message[message.size()]);
    openDriver();
    ASSERT_THROW_MESSAGE(
        driver.processOne(),
        std::invalid_argument,
        "EP: payload too small for embedded message");
}

