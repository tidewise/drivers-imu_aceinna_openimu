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

All other interactions with the unit is using reply/request pairs. The
response is always of the same type than the request. For instance, a pG
packet sent to the unit will be answered with a pG packet, only with a
different payload.

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

## sC - Save configuration to flash

### Query

- No payload

### Reply

- No payload

## z1 - Scaled 9-axis IMU packet

### Query

- No payload

### Reply

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

### Query

- No payload

### Reply

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

### Query

- No payload

### Reply

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

### Query

- No payload

### Reply

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

### Query

- No payload

### Reply

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

### Query

- No payload

### Reply

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

## s1 - IMU Scaled Sensors

### Query

- No payload

### Reply

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
