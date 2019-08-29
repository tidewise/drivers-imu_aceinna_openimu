#include <imu_aceinna_openimu/Driver.hpp>
#include <imu_aceinna_openimu/Protocol.hpp>
#include <iostream>

using namespace std;
using namespace imu_aceinna_openimu;

struct NullStream : std::ostream {
};
static NullStream null_progress;

ostream& Driver::nullStream() {
    return null_progress;
}

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

DeviceInfo Driver::validateDevice(bool allow_bootloader)
{
    auto deviceInfo = readDeviceInfo();
    if (!deviceInfo.bootloader_mode) {
        auto app_version = deviceInfo.app_version;
        auto ins = app_version.substr(0, 3);
        auto tw = app_version.substr(app_version.length() - 2, 2);
        if (ins != "INS" || tw != "TW") {
            close();
            throw UnsupportedDevice("this driver requires the TideWise version "\
                                    "of the INS app, got: " + app_version);
        }
    }
    else if (!allow_bootloader) {
        throw UnsupportedDevice("device is in bootloader mode");
    }

    return deviceInfo;
}

DeviceInfo Driver::readDeviceInfo()
{
    auto packetEnd = protocol::queryDeviceID(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    auto packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    string deviceID = protocol::parseDeviceID(
        mReadBuffer + protocol::PAYLOAD_OFFSET,
        packetSize - protocol::PACKET_OVERHEAD);

    if (deviceID.find("OpenIMU_Bootloader") == 0) {
        return DeviceInfo { true, deviceID, "" };
    }

    packetEnd = protocol::queryAppVersion(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    string appVersion = protocol::parseAppVersion(
        mReadBuffer + protocol::PAYLOAD_OFFSET,
        packetSize - protocol::PACKET_OVERHEAD);
    return DeviceInfo { false, deviceID, appVersion };
}

Status Driver::readStatus()
{
    auto packetEnd = protocol::queryStatus(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    auto packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    return protocol::parseStatus(
        mReadBuffer + protocol::PAYLOAD_OFFSET,
        packetSize - protocol::PACKET_OVERHEAD);
}

Configuration Driver::readConfiguration()
{
    auto packetEnd = protocol::queryConfiguration(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    auto packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    return protocol::parseConfiguration(
        mReadBuffer + protocol::PAYLOAD_OFFSET,
        packetSize - protocol::PACKET_OVERHEAD);
}

template<typename T>
string to_string(T value) {
    return std::to_string(value);
}
string to_string(string value) {
    return value;
}

void Driver::queryReset() {
    auto packetEnd = protocol::queryReset(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
}

void Driver::queryRestoreDefaultConfiguration() {
    auto packetEnd = protocol::queryRestoreDefaultConfiguration(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    auto packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    std::cout << "response size: " << packetSize << std::endl;
}

struct PeriodicMessage {
    char const* name;
    int index;
};

static const PeriodicMessage PERIODIC_MESSAGES[] = {
    { "z1", 2 },
    { "e3", 8 },
    { "i1", 9 }
};

static PeriodicMessage const* findPeriodicMessage(string name) {
    for (auto const& p : PERIODIC_MESSAGES) {
        if (p.name == name) {
            return &p;
        }
    }
    return nullptr;
}

void Driver::writeExtendedPeriodMessageConfiguration(string name, int period)
{
    auto msg = findPeriodicMessage(name);
    if (!msg) {
        throw std::invalid_argument("unknown periodic message " + name);
    }

    return writeExtendedPeriodMessageConfiguration(msg->index, period);
}

void Driver::writeExtendedPeriodMessageConfiguration(int index, int period)
{
    uint8_t paramIndex = 15 + index / 8;
    string current = readConfiguration<string>(paramIndex);
    uint8_t offset = index % 8;
    current.at(offset) = period;
    writeConfiguration(paramIndex, current, true);
}

template <typename T>
void Driver::writeConfiguration(int index, T value, bool validate)
{
    auto packetEnd = protocol::writeConfiguration(mWriteBuffer, index, value);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    int packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    auto status = protocol::parseWriteConfigurationStatus(
        mReadBuffer + protocol::PAYLOAD_OFFSET,
        packetSize - protocol::PACKET_OVERHEAD);

    if (status != protocol::WRITE_STATUS_OK) {
        string error_message;
        if (status == protocol::WRITE_STATUS_INVALID_INDEX) {
            error_message = "invalid parameter index";
        }
        else if (status == protocol::WRITE_STATUS_INVALID_VALUE) {
            error_message = "invalid value " + to_string(value);
        }
        else {
            error_message = "unknown error";
        }
        throw ConfigurationWriteFailed(
            "writing configuration parameter " + to_string(index) + " failed: " +
            error_message);
    }

    if (validate) {
        auto actual = readConfiguration<T>(index);
        if (actual != value) {
            throw ConfigurationWriteFailed(
                "writing configuration parameter " + to_string(index) + " failed. "
                "Current property value is " + to_string(actual) +
                ", expected " + to_string(value));
        }
    }
}
template void Driver::writeConfiguration<int64_t>(int, int64_t, bool);
template void Driver::writeConfiguration<string>(int, string, bool);

void Driver::writeConfiguration(Configuration const& conf, bool validate)
{
    writeConfiguration(3, conf.periodic_packet_type, validate);
    writeConfiguration<int64_t>(4, conf.periodic_packet_rate, validate);
    writeConfiguration<int64_t>(5, conf.acceleration_low_pass_filter, validate);
    writeConfiguration<int64_t>(6, conf.angular_velocity_low_pass_filter, validate);
    writeConfiguration(7, conf.orientation, validate);
}

template<typename T>
T Driver::readConfiguration(int index)
{
    auto packetEnd = protocol::queryConfigurationParameter(mWriteBuffer, index);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    auto packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    return protocol::parseConfigurationParameter<T>(
        mReadBuffer + protocol::PAYLOAD_OFFSET,
        packetSize - protocol::PACKET_OVERHEAD,
        index);
}
template int64_t Driver::readConfiguration<int64_t>(int index);
template string Driver::readConfiguration<string>(int index);

void Driver::saveConfiguration()
{
    auto packetEnd = protocol::queryConfigurationSave(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
}

void Driver::writeBaudrate(int rate)
{
    auto packetEnd = protocol::writeConfiguration<int64_t>(mWriteBuffer, 2, rate);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
}

void Driver::writePeriodicPacketConfiguration(string packet, int rate)
{
    writeConfiguration<string>(3, packet);
    writeConfiguration<int64_t>(4, rate);
}

void Driver::writeUsedSensors(bool magnetometers, bool gps, bool gps_course_as_heading) {
    int64_t field = 0;
    if (magnetometers) field |= 1;
    if (gps) field |= 2;
    if (gps_course_as_heading) field |= 4;
    writeConfiguration<int64_t>(12, field);
}

int Driver::extractPacket(uint8_t const* buffer, size_t bufferSize) const
{
    return protocol::extractPacket(buffer, bufferSize);
}

void Driver::toBootloader()
{
    auto packetEnd = protocol::queryJumpToBootloader(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    close();
}

void Driver::toApp()
{
    auto packetEnd = protocol::queryJumpToApp(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    close();
}

void Driver::writeFirmware(std::vector<uint8_t> const& bin, std::ostream& progress)
{
    unsigned int i = 0;
    while (i < bin.size()) {
        progress << "\r" << i << "/" << bin.size() << std::flush;
        int remaining = bin.size() - i;
        int blockSize = min(remaining, protocol::MAX_APP_BLOCK_SIZE);

        auto packetEnd = protocol::queryAppBlockWrite(
            mWriteBuffer, i, &bin[i], blockSize);
        writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
        readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2,
                         base::Time::fromSeconds(10));
        i += blockSize;
    }
}

void Driver::UpdateResult::add(UpdateType type) {
    if (type != UPDATED_IGNORED) {
        updated |= type;
    }
}
bool Driver::UpdateResult::isUpdated(UpdateType type) const {
    return updated & type;
}

Driver::UpdateResult Driver::processOne() {
    int packetSize = readPacket(mReadBuffer, BUFFER_SIZE);
    UpdateResult result;

    if (mReadBuffer[2] == 'E' && mReadBuffer[3] == 'P') {
        uint8_t const* payload = mReadBuffer + protocol::PAYLOAD_OFFSET;
        uint8_t payloadLen = packetSize - protocol::PACKET_OVERHEAD;
        uint8_t offset = 0;
        while (offset != payloadLen) {
            if (offset + 3 > payloadLen) {
                throw std::invalid_argument("EP: payload too small for embedded message");
            }

            uint8_t len = payload[offset + 2];
            if (offset + 3 + len > payloadLen) {
                throw std::invalid_argument("EP: payload too small for embedded message");
            }

            auto update = processOne(
                payload + offset,
                payload + offset + 3, len);
            result.add(update);

            offset += len + 3;
        }
    }
    else {
        auto update = processOne(mReadBuffer + 2,
                                 mReadBuffer + protocol::PAYLOAD_OFFSET,
                                 packetSize - protocol::PACKET_OVERHEAD);
        result.add(update);
    }

    return result;
}

Driver::UpdateType Driver::processOne(uint8_t const* type, uint8_t const* payload, uint8_t payloadLen) {
    if (type[0] == 'e' && type[1] == '3') {
        mEKFWithCovariance = protocol::parseEKFWithCovariance(payload, payloadLen);
        return UPDATED_STATE;
    }
    else if (type[0] == 'i' && type[1] == '1') {
        mStatus = protocol::parseStatus(payload, payloadLen);
        return UPDATED_STATUS;
    }
    return UPDATED_IGNORED;
}

EKFWithCovariance Driver::getState() const
{
    return mEKFWithCovariance;
}

Status Driver::getIMUStatus() const
{
    return mStatus;
}
