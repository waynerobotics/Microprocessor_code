#pragma once

#include <cstdint>
#include <cstring>

namespace spark {

// SPARK MAX USB CDC binary protocol — fixed 12-byte packets.
// CAN extended ID encoded into bytes [0:3] (little-endian):
//   bits 28:24 device type (0x02), 23:16 manufacturer (0x05),
//   15:12 API class, 11:8 API index, 7:2 device ID, 1:0 reserved.
// Bytes [4:11] are the 8-byte data payload.

static constexpr uint32_t SPARK_DEVICE_TYPE   = 0x02u;
static constexpr uint32_t SPARK_MANUFACTURER  = 0x05u;

static constexpr uint32_t API_CLASS_SETPOINT  = 0x02u;
static constexpr uint32_t API_CLASS_HEARTBEAT = 0x06u;

static constexpr uint32_t API_IDX_DUTY_CYCLE  = 0x00u;
static constexpr uint32_t API_IDX_VELOCITY    = 0x01u;
static constexpr uint32_t API_IDX_POSITION    = 0x02u;
static constexpr uint32_t API_IDX_HEARTBEAT   = 0x02u;

static constexpr uint16_t SPARK_USB_VID = 0x0483u;
static constexpr uint16_t SPARK_USB_PID = 0x5740u;

static constexpr int PACKET_BYTES = 12;

inline uint32_t make_cmd_id(uint32_t api_class, uint32_t api_index,
                            uint32_t device_id = 0u)
{
    return (SPARK_DEVICE_TYPE       << 24u) |
           (SPARK_MANUFACTURER      << 16u) |
           ((api_class  & 0xFu)     << 12u) |
           ((api_index  & 0xFu)     <<  8u) |
           ((device_id  & 0x3Fu)    <<  2u);
}

struct Packet {
    uint8_t bytes[PACKET_BYTES] = {};

    void set_cmd_id(uint32_t id) {
        bytes[0] = static_cast<uint8_t>(id);
        bytes[1] = static_cast<uint8_t>(id >>  8u);
        bytes[2] = static_cast<uint8_t>(id >> 16u);
        bytes[3] = static_cast<uint8_t>(id >> 24u);
    }

    void set_float32(int offset, float v) {
        uint32_t bits;
        std::memcpy(&bits, &v, sizeof(bits));
        bytes[offset + 0] = static_cast<uint8_t>(bits);
        bytes[offset + 1] = static_cast<uint8_t>(bits >>  8u);
        bytes[offset + 2] = static_cast<uint8_t>(bits >> 16u);
        bytes[offset + 3] = static_cast<uint8_t>(bits >> 24u);
    }

    static Packet heartbeat() {
        Packet p;
        p.set_cmd_id(make_cmd_id(API_CLASS_HEARTBEAT, API_IDX_HEARTBEAT));
        return p;
    }

    static Packet position_setpoint(float rotations, uint32_t device_id = 0u) {
        Packet p;
        p.set_cmd_id(make_cmd_id(API_CLASS_SETPOINT, API_IDX_POSITION, device_id));
        p.set_float32(4, rotations);
        return p;
    }
};

}  // namespace spark
