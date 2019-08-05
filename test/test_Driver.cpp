#include <gtest/gtest.h>
#include <imu_aceinna_openimu/Driver.hpp>
#include <iodrivers_base/FixtureGTest.hpp>

using namespace std;
using namespace imu_aceinna_openimu;

struct DriverTest : public ::testing::Test, iodrivers_base::Fixture<Driver> {
};

TEST_F(DriverTest, it_queries_the_device_description_on_open) {
    IODRIVERS_BASE_MOCK();
    EXPECT_REPLY(
        vector<uint8_t>{ 0x55, 0x55, 'p', 'G', 0, 0x5d, 0x5f },
        vector<uint8_t>{ 0x55, 0x55, 'p', 'G', 4, 'a', 'b', 'c', 'd', 0x1f, 0x72 });
    driver.openURI("test://");
    ASSERT_EQ("abcd", driver.getDeviceInfo());
}
