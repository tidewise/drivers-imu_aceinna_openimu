#include <boost/test/unit_test.hpp>
#include <imu_aceinna_openimu/Dummy.hpp>

using namespace imu_aceinna_openimu;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    imu_aceinna_openimu::DummyClass dummy;
    dummy.welcome();
}
