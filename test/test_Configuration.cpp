#include <gtest/gtest.h>
#include <imu_aceinna_openimu/Configuration.hpp>

using namespace imu_aceinna_openimu;

struct ConfigurationTest : public ::testing::Test {};

TEST_F(ConfigurationTest, it_equals_its_copy)
{
    Configuration config;
    Configuration other = config;
    ASSERT_EQ(config, other);
}

TEST_F(ConfigurationTest, it_differs_if_the_periodic_packet_type_is_different)
{
    Configuration config;
    Configuration other = config;
    other.periodic_packet_type = "some";
    ASSERT_NE(config, other);
}