#include <gtest/gtest.h>
#include <imu_aceinna_openimu/Driver.hpp>
#include <imu_aceinna_openimu/Protocol.hpp>
#include <iodrivers_base/FixtureGTest.hpp>

using namespace std;
using namespace imu_aceinna_openimu;

struct DriverTest : public ::testing::Test, iodrivers_base::Fixture<Driver> {
    void openDriver() {
        EXPECT_REPLY(
            vector<uint8_t>{ 0x55, 0x55, 'p', 'G', 0, 0x5d, 0x5f },
            vector<uint8_t>{ 0x55, 0x55, 'p', 'G', 4, 'a', 'b', 'c', 'd', 0x1f, 0x72 });
        EXPECT_REPLY(
            vector<uint8_t>{ 0x55, 0x55, 'g', 'V', 0, 0xab, 0xee },
            vector<uint8_t>{ 0x55, 0x55, 'g', 'V', 4, 'e', 'f', 'g', 'h', 0x75, 0x10 });
        driver.openURI("test://");
    }
};

TEST_F(DriverTest, it_queries_the_device_description_on_open) {
    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'p', 'G', 0, 0x5d, 0x5f },
        vector<uint8_t>{ 0x55, 0x55, 'p', 'G', 4, 'a', 'b', 'c', 'd', 0x1f, 0x72 });
    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'g', 'V', 0, 0xab, 0xee },
        vector<uint8_t>{ 0x55, 0x55, 'g', 'V', 4, 'e', 'f', 'g', 'h', 0x75, 0x10 });
    driver.openURI("test://");

    auto info = driver.getDeviceInfo();
    ASSERT_EQ("abcd", info.device_id);
    ASSERT_EQ("efgh", info.app_version);
}

TEST_F(DriverTest, it_raises_if_the_property_actual_value_differs_from_the_written) {
    IODRIVERS_BASE_MOCK();
    openDriver();

    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'u', 'P', 12, 2, 0, 0, 0,
                         0x20, 0x10, 0, 0, 0, 0, 0, 0, 0xA2, 0x73 },
        vector<uint8_t>{ 0x55, 0x55, 'u', 'P', 0, 0x2c, 0x4b });
    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'g', 'P', 4, 2, 0, 0, 0, 0xa6, 0xd6 },
        vector<uint8_t>{ 0x55, 0x55, 'g', 'P', 12, 2, 0, 0, 0,
                         0x20, 0x11, 0, 0, 0, 0, 0, 0, 0x19, 0x41 });
    ASSERT_THROW({
        try {
            driver.writeConfiguration<int64_t>(2, 0x1020, true);
        }
        catch(ConfigurationWriteFailed& e) {
            ASSERT_EQ("writing configuration parameter 2 failed. "
                      "Current property value is 4384, expected 4128", string(e.what()));
            throw;
        }
    }, ConfigurationWriteFailed);
}
