#include <gtest/gtest.h>
#include <imu_aceinna_openimu/NMEAPublisher.hpp>

#include <sys/socket.h>
#include <sys/types.h>

using namespace imu_aceinna_openimu;
using namespace std;
using base::Time;

class RawIODriver : public iodrivers_base::Driver {
    int extractPacket(uint8_t const* buffer, size_t buffer_size) const
    {
        return 0;
    }

public:
    RawIODriver()
        : Driver(256)
    {
    }
};

struct NMEAPublisherTest : public ::testing::Test {
    NMEAPublisher publisher;
    RawIODriver device;
    RawIODriver forward;
    RawIODriver nmea;

    NMEAPublisherTest()
        : publisher("GP")
    {
        auto fds = openSocketPair();
        publisher.openDevice("fd://" + to_string(fds.first));
        device.openURI("fd://" + to_string(fds.second));

        fds = openSocketPair();
        publisher.openForward("fd://" + to_string(fds.first), Time::fromMilliseconds(10));
        forward.openURI("fd://" + to_string(fds.second));

        fds = openSocketPair();
        publisher.openNMEA("fd://" + to_string(fds.first));
        nmea.openURI("fd://" + to_string(fds.second));
    }

    pair<int, int> openSocketPair()
    {
        int fds[2];
        if (socketpair(AF_UNIX, SOCK_DGRAM, 0, fds) == -1) {
            throw iodrivers_base::UnixError("cannot create socket pair");
        }
        return make_pair(fds[0], fds[1]);
    }
};

TEST_F(NMEAPublisherTest, it_forwards_messages_from_the_device_to_the_forward)
{
    vector<uint8_t> msg{0x55, 0x55, 'p', 'G', 0, 0x5d, 0x5f};
    device.writePacket(msg.data(), msg.size());
    publisher.process(Time::fromSeconds(1));
    vector<uint8_t> buffer(32768, 0);
    ASSERT_EQ(msg.size(),
        forward.readRaw(buffer.data(), msg.size(), Time::fromSeconds(1)));
    ASSERT_EQ(msg, vector<uint8_t>(buffer.begin(), buffer.begin() + msg.size()));
    ASSERT_EQ(0, nmea.readRaw(buffer.data(), buffer.size(), Time::fromMilliseconds(100)));
}

TEST_F(NMEAPublisherTest, it_forwards_messages_from_the_forward_to_the_device)
{
    vector<uint8_t> msg{1, 2, 3, 4, 5};
    forward.writePacket(msg.data(), msg.size());
    publisher.process(Time::fromSeconds(1));
    vector<uint8_t> buffer(32768, 0);
    ASSERT_EQ(msg.size(),
        device.readRaw(buffer.data(), msg.size(), Time::fromSeconds(1)));
    ASSERT_EQ(msg, vector<uint8_t>(buffer.begin(), buffer.begin() + msg.size()));
    ASSERT_EQ(0, nmea.readRaw(buffer.data(), buffer.size(), Time::fromMilliseconds(100)));
}

// clang-format off
// Sample taken from an IMU log. Yaw = 91.6
// Log ID: 090aeb8eadaebeb3f36c992967ca, Time: 1665004477.344012
// Q_nwu=-0.19867369532585144 -0.18287518620491025 0.67453211545944214 0.68708944320678711
static vector<uint8_t> IMU_E4_MESSAGE{
    85, 85, 101, 52, 97, 97, 207, 31, 0, 4, 162, 67, 59, 62,
    30, 113, 75, 190, 24, 229, 47, 191, 35, 174, 44, 63, 18, 182, 130, 191, 16,
    92, 201, 62, 74, 168, 105, 191, 195, 107, 147, 189, 64, 213, 230, 59, 126,
    255, 20, 189, 155, 52, 130, 155, 216, 234, 54, 192, 78, 25, 210, 29, 218,
    149, 69, 192, 0, 0, 128, 227, 217, 241, 252, 191, 109, 64, 26, 190, 59, 223,
    15, 62, 111, 236, 168, 188, 102, 197, 202, 191, 217, 219, 180, 60, 13, 196,
    37, 64, 169, 31, 203, 190, 64, 134};
// clang-format on

TEST_F(NMEAPublisherTest, it_generates_a_valid_HDT_message_based_on_a_e4_message)
{
    publisher.selectNMEAMessages(NMEA_PUBLISH_HDT);
    device.writePacket(IMU_E4_MESSAGE.data(), IMU_E4_MESSAGE.size());
    publisher.process(Time::fromSeconds(1));
    vector<uint8_t> buffer(32768, 0);
    ASSERT_EQ(IMU_E4_MESSAGE.size(),
        forward.readRaw(buffer.data(), IMU_E4_MESSAGE.size(), Time::fromSeconds(1)));
    ASSERT_EQ(IMU_E4_MESSAGE,
        vector<uint8_t>(buffer.begin(), buffer.begin() + IMU_E4_MESSAGE.size()));

    int size = nmea.readRaw(buffer.data(), buffer.size(), Time::fromMilliseconds(100));
    string msg(reinterpret_cast<char const*>(buffer.data()),
        reinterpret_cast<char const*>(buffer.data()) + size);
    ASSERT_EQ("$GPHDT,148.7,T*3F\r\n", msg);
}

TEST_F(NMEAPublisherTest, it_generates_a_valid_ZDA_message)
{
    base::Time t = base::Time::fromMicroseconds(1749671780052521ULL);
    auto sentence = publisher.getZDASentence(t);
    ASSERT_EQ("$GPZDA,195620.052,11,06,2025,00,00*5B\r\n", sentence);
}

TEST_F(NMEAPublisherTest, it_publishes_ZDA_messages_if_selected)
{
    publisher.selectNMEAMessages(NMEA_PUBLISH_ZDA);
    device.writePacket(IMU_E4_MESSAGE.data(), IMU_E4_MESSAGE.size());
    publisher.process(Time::fromSeconds(1));

    vector<uint8_t> buffer(32768, 0);
    int size = nmea.readRaw(buffer.data(), buffer.size(), Time::fromMilliseconds(100));
    string msg(reinterpret_cast<char const*>(buffer.data()),
        reinterpret_cast<char const*>(buffer.data()) + size);
    ASSERT_EQ("$GPZDA", msg.substr(0, 6));
}

TEST_F(NMEAPublisherTest, it_ignores_an_invalid_orientation)
{
    PeriodicUpdate update;
    update.rbs = base::samples::RigidBodyState::invalid();
    publisher.publishNMEA(update);

    uint8_t buffer[32768];
    ASSERT_EQ(0, nmea.readRaw(buffer, 32768, Time::fromMilliseconds(100)));
}
