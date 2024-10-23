#include <imu_aceinna_openimu/Driver.hpp>
#include <imu_aceinna_openimu/Protocol.hpp>
#include <imu_aceinna_openimu/Parameter.hpp>
#include <iostream>

using namespace std;
using namespace imu_aceinna_openimu;

struct NullStream : std::ostream {};
static NullStream null_progress;

ostream& Driver::nullStream()
{
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

int Driver::readPacketsUntil(uint8_t* buffer,
    int bufferSize,
    uint8_t const* command,
    base::Time timeout)
{
    base::Time deadline = base::Time::now() + timeout;
    do {
        auto now = base::Time::now();
        auto singleReadTimeout = now < deadline ? deadline - now : base::Time();
        int packetSize = readPacket(buffer, bufferSize, singleReadTimeout);
        if (buffer[2] == 0 && buffer[3] == 0)
            throw std::invalid_argument("IMU reports an invalid packet");
        if (buffer[2] == command[0] && buffer[3] == command[1])
            return packetSize;
    } while (deadline > base::Time::now());

    char const* cmd_chars = reinterpret_cast<char const*>(command);
    throw iodrivers_base::TimeoutError(iodrivers_base::TimeoutError::PACKET,
        "packets were received, but none were of the expected type " +
            string(cmd_chars, cmd_chars + 2) + " received in the expected timeout");

    return 0; // never reached
}

DeviceInfo Driver::validateDevice(bool allow_bootloader)
{
    auto deviceInfo = readDeviceInfo();
    if (!deviceInfo.bootloader_mode) {
        auto app_version = deviceInfo.app_version;
        bool ins = app_version.find("INS") != string::npos;
        if (!ins) {
            close();
            throw UnsupportedDevice(
                "this driver requires the INS app, got " + app_version);
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
    string deviceID = protocol::parseDeviceID(mReadBuffer + protocol::PAYLOAD_OFFSET,
        packetSize - protocol::PACKET_OVERHEAD);

    if (deviceID.find("OpenIMU_Bootloader") == 0) {
        return DeviceInfo{true, deviceID, ""};
    }

    packetEnd = protocol::queryAppVersion(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    string appVersion = protocol::parseAppVersion(mReadBuffer + protocol::PAYLOAD_OFFSET,
        packetSize - protocol::PACKET_OVERHEAD);
    return DeviceInfo{false, deviceID, appVersion};
}

Configuration Driver::readConfiguration()
{
    auto packetEnd = protocol::queryConfiguration(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    auto packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    return protocol::parseConfiguration(mReadBuffer + protocol::PAYLOAD_OFFSET,
        packetSize - protocol::PACKET_OVERHEAD);
}

template <typename T> string to_string(T value)
{
    return std::to_string(value);
}
string to_string(string value)
{
    return value;
}

void Driver::queryReset()
{
    auto packetEnd = protocol::queryReset(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
}

Configuration Driver::reset(int tries)
{
    queryReset();

    for (int i = 0; i < tries; ++i) {
        try {
            return readConfiguration();
        }
        catch(iodrivers_base::TimeoutError&) {
        }
    }

    throw ResetFailedError(
        "failed to communicate with the IMU after reset, tried " +
        std::to_string(tries) + " times"
    );
}

void Driver::queryRestoreDefaultConfiguration()
{
    auto packetEnd = protocol::queryRestoreDefaultConfiguration(mWriteBuffer);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    auto packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    std::cout << "response size: " << packetSize << std::endl;
}

template <typename T> void Driver::writeConfiguration(int index, T value, bool validate)
{
    auto packetEnd = protocol::writeConfiguration(mWriteBuffer, index, value);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    int packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    auto status =
        protocol::parseWriteConfigurationStatus(mReadBuffer + protocol::PAYLOAD_OFFSET,
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
        throw ConfigurationWriteFailed("writing configuration parameter " +
                                       to_string(index) + " failed: " + error_message);
    }

    if (validate) {
        auto actual = readConfiguration<T>(index);
        if (actual != value) {
            throw ConfigurationWriteFailed(
                "writing configuration parameter " + to_string(index) +
                " failed. "
                "Current property value is " +
                to_string(actual) + ", expected " + to_string(value));
        }
    }
}
template void Driver::writeConfiguration<int64_t>(int, int64_t, bool);
template void Driver::writeConfiguration<string>(int, string, bool);
template void Driver::writeConfiguration<double>(int, double, bool);

void Driver::writeConfiguration(Configuration const& conf, bool validate)
{
    writeConfiguration(3, conf.periodic_packet_type, validate);
    writeConfiguration<int64_t>(4, conf.periodic_packet_rate, validate);
    writeConfiguration<int64_t>(5, conf.acceleration_low_pass_filter, validate);
    writeConfiguration<int64_t>(6, conf.angular_velocity_low_pass_filter, validate);
    writeConfiguration(7, conf.orientation, validate);
}

template <typename T> T Driver::readConfiguration(int index)
{
    auto packetEnd = protocol::queryConfigurationParameter(mWriteBuffer, index);
    writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
    auto packetSize = readPacketsUntil(mReadBuffer, BUFFER_SIZE, mWriteBuffer + 2);
    return protocol::parseConfigurationParameter<T>(mReadBuffer +
                                                        protocol::PAYLOAD_OFFSET,
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

void Driver::writeAccelerationLowPassFilter(int64_t rate)
{
    writeConfiguration<int64_t>(5, rate);
}

void Driver::writeAngularVelocityLowPassFilter(int64_t rate)
{
    writeConfiguration<int64_t>(6, rate);
}

void Driver::writeGPSBaudrate(int baudrate)
{
    writeConfiguration<int64_t>(8, baudrate);
}

void Driver::writeGPSProtocol(GPSProtocol protocol)
{
    writeConfiguration<int64_t>(9, protocol);
}

void Driver::writeLeverArm(base::Vector3d const& arm)
{
    writeConfiguration<double>(14, arm.x());
    writeConfiguration<double>(15, arm.y());
    writeConfiguration<double>(16, arm.z());
}

void Driver::writePointOfInterest(base::Vector3d const& point)
{
    writeConfiguration<double>(17, point.x());
    writeConfiguration<double>(18, point.y());
    writeConfiguration<double>(19, point.z());
}

void Driver::writeMagneticCalibration(MagneticCalibration const& calibration)
{
    writeConfiguration<double>(10, calibration.hard_iron.x());
    writeConfiguration<double>(11, calibration.hard_iron.y());
    writeConfiguration<double>(12, calibration.soft_iron_ratio);
    writeConfiguration<double>(13, calibration.soft_iron_angle.getRad());
}

void Driver::writeRTKHeading2MagHeading(base::Angle const& angle)
{
    writeConfiguration<double>(CONF_RTK_HEADING_TO_MAG_HEADING, angle.getRad());
}

void Driver::writeOrientation(std::string const& orientation)
{
    writeConfiguration(CONF_ORIENTATION, orientation);
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

        auto packetEnd =
            protocol::queryAppBlockWrite(mWriteBuffer, i, &bin[i], blockSize);
        writePacket(mWriteBuffer, packetEnd - mWriteBuffer);
        readPacketsUntil(mReadBuffer,
            BUFFER_SIZE,
            mWriteBuffer + 2,
            base::Time::fromSeconds(10));
        i += blockSize;
    }
}

bool Driver::processOne()
{
    int packetSize = readPacket(mReadBuffer, BUFFER_SIZE);
    return processOne(mReadBuffer + 2,
        mReadBuffer + protocol::PAYLOAD_OFFSET,
        packetSize - protocol::PACKET_OVERHEAD);
}

bool Driver::processOne(uint8_t const* type, uint8_t const* payload, uint8_t payloadLen)
{
    if (type[0] == 'e' && type[1] == '2') {
        mPeriodicUpdate = protocol::parseE2Output(payload, payloadLen);
        return true;
    }
    else if (type[0] == 'e' && type[1] == '4') {
        mPeriodicUpdate = protocol::parseE4Output(payload, payloadLen);
        return true;
    }
    else if (type[0] == 'e' && type[1] == '5') {
        mPeriodicUpdate = protocol::parseE5Output(payload, payloadLen);
        return true;
    }
    return false;
}

PeriodicUpdate Driver::getLastPeriodicUpdate() const
{
    return mPeriodicUpdate;
}

Status Driver::getIMUStatus() const
{
    return mStatus;
}
