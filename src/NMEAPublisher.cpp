#include <base/Time.hpp>
#include <imu_aceinna_openimu/NMEAPublisher.hpp>
#include <imu_aceinna_openimu/Protocol.hpp>
#include <iomanip>

using namespace imu_aceinna_openimu;
using namespace std;
using base::Time;

NMEAPublisher::NMEAPublisher(string const& talker)
    : m_talker(talker)
{
    m_buffer.resize(BUFFER_SIZE);
}

void NMEAPublisher::openDevice(std::string const& uri)
{
    m_device.openURI(uri);
    m_device.setReadTimeout(Time::fromSeconds(1));
    m_device.setWriteTimeout(Time::fromSeconds(1));
}
void NMEAPublisher::openForward(std::string const& uri, base::Time const& read_timeout)
{
    m_forward_read_timeout = read_timeout;
    m_forward.openURI(uri);
    m_forward.setReadTimeout(Time::fromSeconds(1));
    m_forward.setWriteTimeout(Time::fromSeconds(1));
}
void NMEAPublisher::openNMEA(std::string const& uri)
{
    m_nmea.openURI(uri);
    m_nmea.setReadTimeout(Time::fromSeconds(1));
    m_nmea.setWriteTimeout(Time::fromSeconds(1));
}

int NMEAPublisher::forwardToDevice()
{
    int size = 0;
    try {
        size =
            m_forward.readRaw(m_buffer.data(), m_buffer.size(), m_forward_read_timeout);
    }
    catch (iodrivers_base::TimeoutError&) {
    }

    if (size != 0) {
        m_device.writePacket(m_buffer.data(), size);
    }
    return size;
}

pair<bool, PeriodicUpdate> NMEAPublisher::forwardFromDevice()
{
    int packet_size = m_device.readPacket(m_buffer.data(), m_buffer.size());
    m_forward.writePacket(m_buffer.data(), packet_size);

    auto payload = m_buffer.data() + protocol::PAYLOAD_OFFSET;
    auto payload_size = packet_size - protocol::PACKET_OVERHEAD;
    if (m_buffer[2] == 'e' && m_buffer[3] == '2') {
        return make_pair(true, protocol::parseE2Output(payload, payload_size));
    }
    else if (m_buffer[2] == 'e' && m_buffer[3] == '4') {
        return make_pair(true, protocol::parseE4Output(payload, payload_size));
    }

    return make_pair(false, PeriodicUpdate());
}

static uint8_t computeNMEAChecksum(string const& str)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < str.length(); ++i) {
        checksum ^= str.at(i);
    }
    return checksum;
}

void NMEAPublisher::selectNMEAMessages(uint32_t messages) {
    m_messages = messages;
}

void NMEAPublisher::publishNMEA(PeriodicUpdate const& update)
{
    string content;
    if (m_messages & NMEA_PUBLISH_HDT) {
        content += getHDTSentence(update.rbs);
    }

    if (!content.empty()) {
        m_nmea.writePacket(reinterpret_cast<uint8_t const*>(content.data()),
            content.length());
    }
}

string NMEAPublisher::getHDTSentence(base::samples::RigidBodyState const& rbs) {
    if (base::isUnknown(rbs.orientation.w())) {
        return "";
    }

    double heading = -rbs.getYaw();
    if (heading < 0) {
        heading += 2 * M_PI;
    }
    heading = heading * 180 / M_PI;

    ostringstream str;
    str << "HDT," << fixed << setprecision(1) << heading << ",T";
    return makeNMEASentence(str.str());
}

string NMEAPublisher::makeNMEASentence(string const& content) {
    stringstream str;
    str << m_talker << content;
    uint8_t checksum = computeNMEAChecksum(str.str());

    str << "*" << setw(2) << setfill('0') << hex << uppercase
        << static_cast<int>(checksum);

    return "$" + str.str() + "\r\n";
}

bool NMEAPublisher::process(base::Time const& timeout)
{
    int device_fd = m_device.getFileDescriptor();
    int forward_fd = m_forward.getFileDescriptor();

    fd_set set;
    FD_ZERO(&set);
    FD_SET(device_fd, &set);
    FD_SET(forward_fd, &set);

    timeval timeout_spec = {static_cast<time_t>(timeout.toSeconds()),
        suseconds_t(timeout.toMicroseconds() % 1000000ULL)};

    int select_nfd = std::max(device_fd, forward_fd) + 1;
    int ret = select(select_nfd, &set, NULL, NULL, &timeout_spec);
    if (ret < 0 && errno != EINTR)
        throw iodrivers_base::UnixError("forward(): error in select()");
    else if (ret == 0)
        return false;

    if (FD_ISSET(device_fd, &set)) {
        auto update = forwardFromDevice();
        if (update.first) {
            publishNMEA(update.second);
        }
    }

    if (FD_ISSET(forward_fd, &set)) {
        forwardToDevice();
    }

    return true;
}

void NMEAPublisher::configureIMU()
{
    m_device.writePeriodicPacketConfiguration("e4", 10);
}