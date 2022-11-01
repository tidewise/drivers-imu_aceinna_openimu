# General Protocol Description

## Packet format

| Offset | Length | Description |
|--------|--------|-------------|
| 0 | 2 | Packet start code, always 0x55 0x55 |
| 2 | 2 | Command or reply code, two ASCII characters |
| 4 | 1 | Payload length (N) |
| 5 | N | <PAYLOAD BYTES> of Payload length
| 5 + N | 1 | MSB of the packet CRC |
| 5 + N + 1 | 1 | LSB of the packet CRC, excluded the packet start code (bytes 2 to 5 + N) |

## CRC

The CRC is computed on bytes 2 to 5 + N, that is the command, payload length and payload.

The CRC is a 16-bit CRC-CCITT with an initialization value of 0x1D0F;

The following C++ code is a valid implementation of the CRC algorithm

~~~
uint16_t protocol::crc(uint8_t const* begin, uint8_t const* end)
{
    uint32_t crc = 0x1D0F;
    for (auto it = begin; it != end; ++it) {
        crc = crc ^ (static_cast<uint16_t>(*it) << 8);
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = (crc << 1);
        }
        crc = crc & 0xffff;
    }

    return crc;
}
~~~

Example: pG query

| Offset | Value | Description |
|---|------|-------------------|
| 0 | 0x55 | Packet start code |
| 1 | 0x55 | Packet start code |
| 2 | 0x70 | p |
| 3 | 0x47 | G |
| 4 | 0x00 | Payload length |
| 5 | 0x5D | MSB of CRC |
| 6 | 0x5F | LSB of CRC |

In the payload, all values are least significant byte first.

String payloads are ASCII strings of the expected payload length. They can't
be mixed with other payload types.

## Request/Reply and Periodic Packet

The firmware supports having a single periodic packet, that is configured
through the Periodic Packet Type and Periodic Packet Rate configuration
parameters.

We introduced the EP periodic packet to make this mechanism more flexible. See
EP packet documentation for more details.

All other interactions with the unit is using reply/request pairs. The
response is always of the same type than the request. For instance, a pG
packet sent to the unit will be answered with a pG packet, only with a
different payload.

If an invalid/unknown packet type is sent to the unit, it replies with
a packet type of 0x00 0x00.

# Configuration Packets

## Configuration Parameters

| Index | Type | Description |
|-------|------|-------------|
| 0 | uint64 | Data CRC |
| 1 | uint64 | Data Size |
| 2 | int64 | Baud Rate (230400, 115200, 57600, 38400) |
| 3 | char[8] | Periodic Packet Type |
| 4 | int64 | Periodic Packet Rate (200,1 00, 50, 20, 10, 5, 2, 0) |
| 5 | int64 | Accel low-pass filter (50, 40, 25, 20, 10, 5, 2) |
| 6 | int64 | Angular velocity low-pass filter (50, 40, 25, 20, 10, 5, 2) |
| 7 | char[8] | Orientation ("+X+Y+Z") |
| 8 | int64_t | GPS UART Baudrate |
| 9 | int64_t | GPS Protocol |
| 10 | float[2] | Hard iron X and Y |
| 11 | float[2] | Soft iron ratio and angle |
| 12 | int64_t | Enabled sensors |
| 20 | char[8] | Packet periods (messages 0 to 7) |
| 28 | char[8] | Packet periods (messages 8 to 15) |

Orientation specifies the forward, right and down axis, encoded with a sign (+ or -)
and an axis name (X, Y, Z). For instance, "+X-Y-Z" would mean:

- forward axis is positive X
- right axis is negative Y
- up axis is positive Z

Note that the unit does not verify that the system is right-handed. Don't
know what would happen if it was not.

Valid GPS protocols are:

| Protocol | Value |
|----------|-------|
| UBlox Binary | 0 |
| Novatel Binary | 1 |
| Novatel ASCII | 2 |
| NMEA0183 | 3 |
| SIRF Binary | 4 |

Used sensors configuration:

| Bit | Description |
|----------|-------|
| 0 | Magnetometers |
| 1 | GPS |
| 2 | GPS Course used as heading |

## Algorithm States

| Value | Description |
|-------|-------------|
| 0 | Stabilize |
| 1 | Initialize |
| 2 | High-gain AHRS |
| 3 | Low-gain AHRS |
| 4 | INS |

## pG - Get device serial number and factory ID

### Query

- No payload

### Reply

| Type   | Description |
|--------|--------------------------|
| string | Device ID and serial number |

## gV - get user app version

### Query

- No payload

### Reply

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | string | User version |

## gS - Get Status

### Query

- No payload

### Reply

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | uint32 | GPS time of week (ms) |
| 4 | uint32 | Extended periodic packet overflows |
| 8 | uint32 | GPS update count |
| 12 | uint32 | time at last valid GPS message (ms) |
| 16 | uint32 | time at last valid GPS position (ms) |
| 20 | uint32 | time at last valid GPS velocity (ms) |
| 24 | uint32 | number of bytes received on the GPS UART |
| 28 | uint16 | number of overflows while parsing the GPS UART |
| 30 | uint16 | HDOP (0.1 scaling) |
| 32 | uint8  | temperature (C) |
| 33 | uint8  | flags |

Flags format (least significant to most significant):

| Length (bit) | Description |
|--------------|-------------|
| 3 | Algorithm state (see beginning of section) |
| 1 | Still switch |
| 1 | Turn switch |
| 1 | Course is used as heading |

## gA - Get all configuration

### Query

- No payload

### Reply

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | uint64 | Data CRC |
| 8 | uint64 | Data Size |
| 16 | int64 | Baud Rate (115200, 57600, 230400, 38400) |
| 24 | char[8] | Periodic Packet Type |
| 32 | int64 | Periodic Packet Rate (200,1 00, 50, 20, 10, 5, 2, 0) |
| 40 | int64 | Accel low-pass filter (50, 40, 25, 20, 10, 5, 2) |
| 48 | int64 | Angular velocity low-pass filter (50, 40, 25, 20, 10, 5, 2) |
| 56 | char[8] | Orientation ("+X+Y+Z") |
| 64 | int64_t | GPS UART Baudrate |
| 72 | int64_t | GPS Protocol |
| 80 | float   | Hard iron X |
| 84 | float   | Hard iron Y |
| 88 | float   | Soft iron ratio |
| 92 | float   | Soft iron angle |
| 96 | int64_t | Enabled sensors |

## gP - Get single parameter

### Query

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0      | int32  | Parameter Index |

### Reply

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | int32 | Parameter Index |
| 4 | type of the queried parameter | Parameter value |

## uP - Set single parameter

### Query

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0      | int32  | Parameter Index |
| 4 | type of the queried parameter | Parameter value |

### Reply

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | int32 | Parameter Index |
| 4 | int32 | Result |

Result may be 0 (OK), -1 (INVALID_PARAM) or -2 (INVALID_VALUE)

## sC - Save configuration to flash

### Query

- No payload

### Reply

- No payload

## rD - Restore defaults

Loads defaults to memory and save them to flash

### Query

- No payload

### Reply

- No payload

## rS - Reset

### Query

- No payload

### Reply

- This message does not send any reply

# Data packets

Data packets are only available as periodic packets.

## z1 - Scaled 9-axis IMU packet

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | uint32 | Time in seconds |
| 4 | float | Acceleration along X (m/s/s) |
| 8 | float | Acceleration along Y (m/s/s) |
| 12 | float | Acceleration along Z (m/s/s) |
| 16 | float | Angular velocity around X (deg/s) |
| 20 | float | Angular velocity around Y (deg/s) |
| 24 | float | Angular velocity around Z (deg/s) |
| 28 | float | Magnetic field in the X direction (Gauss) |
| 32 | float | Magnetic field in the Y direction (Gauss) |
| 36 | float | Magnetic field in the Z direction (Gauss) |

## z3 - Scaled 6-axis IMU packet

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | uint32 | Time in milliseconds |
| 4 | float | Acceleration along X (m/s/s) |
| 8 | float | Acceleration along Y (m/s/s) |
| 12 | float | Acceleration along Z (m/s/s) |
| 16 | float | Angular velocity around X (rad/s) |
| 20 | float | Angular velocity around Y (rad/s) |
| 24 | float | Angular velocity around Z (rad/s) |

## a1 - VG Output Message with Flags

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | uint32 | Time in milliseconds |
| 4 | double | Time in seconds |
| 12 | float | Roll (rad) |
| 16 | float | Pitch (rad) |
| 20 | float | Yaw (rad) |
| 24 | float | Angular velocity around X (rad/s) |
| 28 | float | Angular velocity around Y (rad/s) |
| 32 | float | Angular velocity around Z (rad/s) |
| 36 | float | Acceleration along X (m/s/s) |
| 40 | float | Acceleration along Y (m/s/s) |
| 44 | float | Acceleration along Z (m/s/s) |
| 48 | uint8 | Operating mode |
| 52 | uint8 | linAccSw |
| 56 | uint8 | turnSw |

## a2 - VG Output Message without Flags

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | uint32 | Time in milliseconds |
| 4 | double | Time in seconds |
| 12 | float | Roll (rad) |
| 16 | float | Pitch (rad) |
| 20 | float | Yaw (rad) |
| 24 | float | Angular velocity around X (rad/s) |
| 28 | float | Angular velocity around Y (rad/s) |
| 32 | float | Angular velocity around Z (rad/s) |
| 36 | float | Acceleration along X (m/s/s) |
| 40 | float | Acceleration along Y (m/s/s) |
| 44 | float | Acceleration along Z (m/s/s) |

## e1 - VG/AHRS Output Message

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | uint32 | Time in milliseconds |
| 4 | double | Time in seconds |
| 12 | float | Roll (rad) |
| 16 | float | Pitch (rad) |
| 20 | float | Yaw (rad) |
| 24 | float | Acceleration along X (g) |
| 28 | float | Acceleration along Y (g) |
| 32 | float | Acceleration along Z (g) |
| 24 | float | Angular velocity around X (deg/s) |
| 28 | float | Angular velocity around Y (deg/s) |
| 32 | float | Angular velocity around Z (deg/s) |
| 36 | float | Angular velocity bias X (deg/s) |
| 40 | float | Angular velocity bias Y (deg/s) |
| 44 | float | Angular velocity bias Z (deg/s) |
| 48 | float | Magnetic field X (Gauss) |
| 52 | float | Magnetic field Y (Gauss) |
| 56 | float | Magnetic field Z (Gauss) |
| 60 | uint8 | Operating mode |
| 64 | uint8 | linAccSw |
| 68 | uint8 | turnSw |

## e2 - INS Output Message

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | uint32 | Time in milliseconds |
| 4 | double | Time in seconds |
| 12 | float | Roll (rad) |
| 16 | float | Pitch (rad) |
| 20 | float | Yaw (rad) |
| 24 | float | Acceleration along X (g) |
| 28 | float | Acceleration along Y (g) |
| 32 | float | Acceleration along Z (g) |
| 36 | float | Acceleration bias along X (g) |
| 40 | float | Acceleration bias along Y (g) |
| 44 | float | Acceleration bias along Z (g) |
| 48 | float | Angular velocity around X (deg/s) |
| 52 | float | Angular velocity around Y (deg/s) |
| 56 | float | Angular velocity around Z (deg/s) |
| 60 | float | Angular velocity bias X (deg/s) |
| 64 | float | Angular velocity bias Y (deg/s) |
| 68 | float | Angular velocity bias Z (deg/s) |
| 72 | float | Velocity North (m/s) |
| 76 | float | Velocity East (m/s) |
| 80 | float | Velocity Down (m/s) |
| 84 | float | Magnetic field X (Gauss) |
| 88 | float | Magnetic field Y (Gauss) |
| 92 | float | Magnetic field Z (Gauss) |
| 96 | double | Latitude (deg) |
| 104 | double | Longitude (deg) |
| 112 | double | Altitude (m) |
| 120 | uint8 | Operating mode |
| 121 | uint8 | linAccSw |
| 122 | uint8 | turnSw |

## e3 - INS Output Message with Covariances

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | uint32 | GPS time of week (ms) |
| 4 | float | Roll (deg) |
| 8 | float | Pitch (deg) |
| 12 | float | Yaw (deg) |
| 16 | float | Roll Covariance (deg^2) |
| 20 | float | Pitch Covariance (deg^2) |
| 24 | float | Yaw Covariance (deg^2) |
| 28 | float | Acceleration along X (g) |
| 32 | float | Acceleration along Y (g) |
| 36 | float | Acceleration along Z (g) |
| 40 | float | Acceleration covarianceX (g^2) |
| 44 | float | Acceleration covarianceY (g^2) |
| 48 | float | Acceleration covarianceZ (g^2) |
| 52 | float | Angular velocity around X (deg/s) |
| 56 | float | Angular velocity around Y (deg/s) |
| 60 | float | Angular velocity around Z (deg/s) |
| 64 | float | Angular velocity covariance X ((deg/s)^2) |
| 68 | float | Angular velocity covariance Y ((deg/s)^2) |
| 72 | float | Angular velocity covariance Z ((deg/s)^2) |
| 76 | float | Velocity North (m/s) |
| 80 | float | Velocity East (m/s) |
| 84 | float | Velocity Down (m/s) |
| 88 | float | Velocity North covariance (Gauss^2) |
| 92 | float | Velocity Est covariance (Gauss^2) |
| 96 | float | Velocity Down covariance (Gauss^2) |
| 100 | double | Latitude (deg) |
| 108 | double | Longitude (deg) |
| 116 | double | Altitude (m) |
| 124 | float | Position covariance N (m^2) |
| 128 | float | Position covariance E (m^2) |
| 132 | float | Position covariance D (m^2) |
| 136 | uint8 | Status byte |

Status byte (least significant to most significant):

| Length (bit) | Description |
|--------------|-------------|
| 3 | Algorithm state (see beginning of section) |
| 1 | Still switch |
| 1 | Turn switch |
| 1 | Course is used as heading |

## e4 - INS Output Message

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | uint32 | GPS time of week (ms) |
| 4 | uint8 | Filter Flags (see below) |
| 5 | float | imu_ned2ned_rot - q.w |
| 9 | float | imu_ned2ned_rot - q.x |
| 13 | float | imu_ned2ned_rot - q.y |
| 17 | float | imu_ned2ned_rot - q.z |
| 21 | float | imu_ned2ned_angv - x (deg) |
| 25 | float | imu_ned2ned_angv - y (deg) |
| 29 | float | imu_ned2ned_angv - z (deg) |
| 33 | float | imu_ned2ned_linv - x |
| 37 | float | imu_ned2ned_linv - y |
| 41 | float | imu_ned2ned_linv - z |
| 49 | double | latitude (deg) |
| 57 | double | longitude (deg) |
| 65 | double | altitude over MSL |
| 65 | float | mag.x |
| 69 | float | mag.y |
| 73 | float | mag.z |
| 77 | float | mag_euler.x |
| 81 | float | mag_euler.y |
| 85 | float | mag_euler.z |
| 89 | float | declination |

Filter state byte (least significant to most significant):

| Length (bit) | Description |
|--------------|-------------|
| 3 | Algorithm state (see beginning of section) |
| 1 | Still switch |
| 1 | Turn switch |
| 1 | Course is used as heading |


## s1 - IMU Scaled Sensors

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | uint32 | Time in milliseconds |
| 4 | double | Time in seconds |
| 12 | float | Acceleration along X (g) |
| 16 | float | Acceleration along Y (g) |
| 20 | float | Acceleration along Z (g) |
| 24 | float | Angular velocity around X (deg/s) |
| 28 | float | Angular velocity around Y (deg/s) |
| 32 | float | Angular velocity around Z (deg/s) |
| 36 | float | Magnetic field X (Gauss) |
| 40 | float | Magnetic field Y (Gauss) |
| 44 | float | Magnetic field Z (Gauss) |
| 48 | float | Degrees (C) |

## i1 - Periodic Information Message

Same as reply to the gS message

## EP - Extended periodic packets

This is a multiplexing of different packet periods. To use the EP packet,
you have to set the periodic packet type to EP and then set the desired
packet periods in the `Packet Periods` fields of the configuration structure.

The period values are in multiples of the EP periodic packet period. For
instance, if the EP packet period is 40 ms (configured rate == 25),
a packet period of 4 means an actual period of 100ms.

The mapping from message to message ID is as follows

| Index | Packet Type |
|-------|-------------|
| 2     | z1 |
| 3     | a1 |
| 4     | a2 |
| 5     | s1 |
| 6     | e1 |
| 7     | e2 |
| 8     | e3 |
| 9     | i1 |

## JI - Jump to Bootloader

Note: communication will switch to 57600 baud *after* JI is acknowledged

**IMPORTANT**: you cannot switch back to app mode after a jump to bootloader.
A new firmware **MUST** be written first.

### Query

- No payload

### Response

- No payload

## JA - Jump to App

### Query

- No payload

## Response

**This packet has no response**

## WA - Write App Block

**NOTE**: the unit usually takes a longer time to respond to the first WA call.
Use a longer timeout.

### Query

| Offset | Type   | Description |
|--------|--------|--------------------------|
| 0 | uint8 | Target address[3] MSB |
| 1 | uint8 | Target address[2] |
| 2 | uint8 | Target address[1] |
| 3 | uint8 | Target address[0] LSB |
| 4 | uint8 | Length of data block in bytes |
| 5 | N bytes | Data - Max 240 bytes |

### Response

- No payload
