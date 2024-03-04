#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>
#include <imu_aceinna_openimu/PeriodicUpdate.hpp>

struct PeriodicUpdateTest : public ::testing::Test {
    PeriodicUpdateTest()
    {
    }
};

TEST_F(PeriodicUpdateTest, it_computes_nan_nwu_position_if_input_is_nan)
{
    imu_aceinna_openimu::PeriodicUpdate update;
    update.filter_state.mode = imu_aceinna_openimu::FilterMode::OPMODE_AHRS_LOW_GAIN;

    update.computeNWUPosition(
        gps_base::UTMConverter(gps_base::UTMConversionParameters()));

    EXPECT_TRUE(std::isnan(update.rbs.position[0]));
    EXPECT_TRUE(std::isnan(update.rbs.position[1]));
    EXPECT_TRUE(std::isnan(update.rbs.position[2]));
}