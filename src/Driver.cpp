#include <imu_aceinna_openimu/Driver.hpp>
#include <imu_aceinna_openimu/Protocol.hpp>

using namespace std;
using namespace imu_aceinna_openimu;


Driver::Driver()
    : iodrivers_base::Driver(BUFFER_SIZE)
{
    static_assert(protocol::MAX_PACKET_SIZE * 10 < Driver::BUFFER_SIZE,
                  "Buffer size needs to contain at least 10 packets");
}

int Driver::readPacketsUntil(uint8_t* buffer, int bufferSize, uint8_t const* command)
{
    return readPacketsUntil(buffer, bufferSize, command, getReadTimeout());
}

int Driver::readPacketsUntil(uint8_t* buffer, int bufferSize, uint8_t const* command,
                             base::Time timeout)
{
    base::Time deadline = base::Time::now() + timeout;
    do
    {
        auto now = base::Time::now();
        auto singleReadTimeout = now < deadline ? deadline - now : base::Time();
        int packetSize = readPacket(buffer, bufferSize, singleReadTimeout);
        if (buffer[2] == command[0] && buffer[3] == command[1])
            return packetSize;
    }
    while (deadline > base::Time::now());

    char const* cmd_chars = reinterpret_cast<char const*>(command);
    throw iodrivers_base::TimeoutError(
        iodrivers_base::TimeoutError::PACKET,
        "packets were received, but none were of the expected type " +
        string(cmd_chars, cmd_chars + 2) + " received in the expected timeout");

    return 0; // never reached
}

void Driver::openURI(std::string const& uri)
{
    iodrivers_base::Driver::openURI(uri);
    mDeviceInfo = queryDeviceInfo();
}

string Driver::queryDeviceInfo()
{
    auto packetEnd = protocol::queryDeviceInfo(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    auto packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    return protocol::parseDeviceInfo(
        mReadBuffer + protocol::PAYLOAD_OFFSET,
        packetSize - protocol::PACKET_OVERHEAD);
}

Configuration Driver::getConfiguration()
{
    auto packetEnd = protocol::queryConfiguration(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    auto packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    return protocol::parseConfiguration(
        mReadBuffer + protocol::PAYLOAD_OFFSET,
        packetSize - protocol::PACKET_OVERHEAD);
}

string Driver::getDeviceInfo() const
{
    return mDeviceInfo;
}

int Driver::extractPacket(uint8_t const* buffer, size_t bufferSize) const
{
    return protocol::extractPacket(buffer, bufferSize);
}
