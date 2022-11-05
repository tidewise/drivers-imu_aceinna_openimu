#ifndef IMU_ACEINNA_OPENIMU_ENDIANNESS_HPP
#define IMU_ACEINNA_OPENIMU_ENDIANNESS_HPP

#include <cstdint>
#include <type_traits>

namespace imu_aceinna_openimu {
    namespace endianness {
        template <typename T>
        uint8_t const* decode(uint8_t const* buffer,
            T& value,
            typename std::enable_if<sizeof(T) == 8>::type* enabler = nullptr)
        {
            uint64_t result = static_cast<uint64_t>(buffer[0]) << 0 |
                              static_cast<uint64_t>(buffer[1]) << 8 |
                              static_cast<uint64_t>(buffer[2]) << 16 |
                              static_cast<uint64_t>(buffer[3]) << 24 |
                              static_cast<uint64_t>(buffer[4]) << 32 |
                              static_cast<uint64_t>(buffer[5]) << 40 |
                              static_cast<uint64_t>(buffer[6]) << 48 |
                              static_cast<uint64_t>(buffer[7]) << 56;
            value = reinterpret_cast<T const&>(result);
            return buffer + sizeof(T);
        }

        template <typename T>
        uint8_t const* decode(uint8_t const* buffer,
            T& value,
            typename std::enable_if<sizeof(T) == 4>::type* enabler = nullptr)
        {
            uint32_t result = static_cast<uint64_t>(buffer[0]) << 0 |
                              static_cast<uint64_t>(buffer[1]) << 8 |
                              static_cast<uint64_t>(buffer[2]) << 16 |
                              static_cast<uint64_t>(buffer[3]) << 24;
            value = reinterpret_cast<T const&>(result);
            return buffer + sizeof(T);
        }

        template <typename T>
        uint8_t const* decode(uint8_t const* buffer,
            T& value,
            typename std::enable_if<sizeof(T) == 2>::type* enabler = nullptr)
        {
            uint16_t result = static_cast<uint16_t>(buffer[0]) << 0 |
                              static_cast<uint16_t>(buffer[1]) << 8;
            value = reinterpret_cast<T const&>(result);
            return buffer + 2;
        }

        template <typename T>
        uint8_t const* decode(uint8_t const* buffer,
            T& value,
            typename std::enable_if<sizeof(T) == 1>::type* enabler = nullptr)
        {
            value = reinterpret_cast<T const&>(buffer[0]);
            return buffer + 1;
        }

        template <typename T>
        uint8_t const* decode(uint8_t const* buffer, T& value, uint8_t const* end)
        {
            if (buffer + sizeof(T) > end) {
                throw std::invalid_argument("buffer too small");
            }
            return decode<T>(buffer, value);
        }

        template <typename T>
        uint8_t* encode(uint8_t* buffer,
            T const& value,
            typename std::enable_if<sizeof(T) == 8>::type* enabler = nullptr)
        {
            uint64_t bytes = reinterpret_cast<uint64_t const&>(value);
            buffer[0] = (bytes >> 0) & 0xFF;
            buffer[1] = (bytes >> 8) & 0xFF;
            buffer[2] = (bytes >> 16) & 0xFF;
            buffer[3] = (bytes >> 24) & 0xFF;
            buffer[4] = (bytes >> 32) & 0xFF;
            buffer[5] = (bytes >> 40) & 0xFF;
            buffer[6] = (bytes >> 48) & 0xFF;
            buffer[7] = (bytes >> 56) & 0xFF;
            return buffer + 8;
        }

        template <typename T>
        uint8_t* encode(uint8_t* buffer,
            T const& value,
            typename std::enable_if<sizeof(T) == 4>::type* enabler = nullptr)
        {
            uint32_t bytes = reinterpret_cast<uint32_t const&>(value);
            buffer[0] = (bytes >> 0) & 0xFF;
            buffer[1] = (bytes >> 8) & 0xFF;
            buffer[2] = (bytes >> 16) & 0xFF;
            buffer[3] = (bytes >> 24) & 0xFF;
            return buffer + 4;
        }

        template <typename T>
        uint8_t* encode(uint8_t* buffer,
            T const& value,
            typename std::enable_if<sizeof(T) == 2>::type* enabler = nullptr)
        {
            uint16_t bytes = reinterpret_cast<uint16_t const&>(value);
            buffer[0] = (bytes >> 0) & 0xFF;
            buffer[1] = (bytes >> 8) & 0xFF;
            return buffer + 2;
        }

        template <typename T>
        uint8_t* encode(uint8_t* buffer,
            T const& value,
            typename std::enable_if<sizeof(T) == 1>::type* enabler = nullptr)
        {
            buffer[0] = reinterpret_cast<uint8_t const&>(value);
            return buffer + 1;
        }
    }
}

#endif