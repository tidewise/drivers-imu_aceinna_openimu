#include <gtest/gtest.h>
#include <imu_aceinna_openimu/Configuration.hpp>

using namespace imu_aceinna_openimu;

struct ConfigurationOrientationTest : public ::testing::Test {
};

TEST_F(ConfigurationOrientationTest, it_is_equal_if_all_three_fields_are) {
    Configuration::Orientation conf0 {
        ORIENTATION_AXIS_PLUS_X, ORIENTATION_AXIS_PLUS_Y, ORIENTATION_AXIS_PLUS_Z
    };
    Configuration::Orientation conf1 {
        ORIENTATION_AXIS_PLUS_X, ORIENTATION_AXIS_PLUS_Y, ORIENTATION_AXIS_PLUS_Z
    };
    ASSERT_EQ(conf0, conf1);
    ASSERT_TRUE(!(conf0 != conf1));
}

TEST_F(ConfigurationOrientationTest, it_is_not_equal_if_forward_differs) {
    Configuration::Orientation conf0 {
        ORIENTATION_AXIS_PLUS_X, ORIENTATION_AXIS_PLUS_Y, ORIENTATION_AXIS_PLUS_Z
    };
    Configuration::Orientation conf1 {
        ORIENTATION_AXIS_PLUS_Y, ORIENTATION_AXIS_PLUS_Y, ORIENTATION_AXIS_PLUS_Z
    };
    ASSERT_NE(conf0, conf1);
    ASSERT_TRUE(conf0 != conf1);
}

TEST_F(ConfigurationOrientationTest, it_is_not_equal_if_right_differs) {
    Configuration::Orientation conf0 {
        ORIENTATION_AXIS_PLUS_X, ORIENTATION_AXIS_PLUS_Y, ORIENTATION_AXIS_PLUS_Z
    };
    Configuration::Orientation conf1 {
        ORIENTATION_AXIS_PLUS_X, ORIENTATION_AXIS_PLUS_X, ORIENTATION_AXIS_PLUS_Z
    };
    ASSERT_NE(conf0, conf1);
    ASSERT_TRUE(conf0 != conf1);
}

TEST_F(ConfigurationOrientationTest, it_is_not_equal_if_down_differs) {
    Configuration::Orientation conf0 {
        ORIENTATION_AXIS_PLUS_X, ORIENTATION_AXIS_PLUS_Y, ORIENTATION_AXIS_PLUS_Z
    };
    Configuration::Orientation conf1 {
        ORIENTATION_AXIS_PLUS_X, ORIENTATION_AXIS_PLUS_Y, ORIENTATION_AXIS_PLUS_Y
    };
    ASSERT_NE(conf0, conf1);
    ASSERT_TRUE(conf0 != conf1);
}

TEST_F(ConfigurationOrientationTest, it_formats_itself_to_string) {
    Configuration::Orientation confplus {
        ORIENTATION_AXIS_PLUS_X, ORIENTATION_AXIS_PLUS_Y, ORIENTATION_AXIS_PLUS_Z
    };
    ASSERT_EQ("+X+Y+Z", to_string(confplus));
    Configuration::Orientation confminus {
        ORIENTATION_AXIS_MINUS_X, ORIENTATION_AXIS_MINUS_Y, ORIENTATION_AXIS_MINUS_Z
    };
    ASSERT_EQ("-X-Y-Z", to_string(confminus));
}