#pragma once
#include <cstdint>
#include <cstddef>

#define TELEMETRY_SYNC1 0xAA
#define TELEMETRY_SYNC2 0x55

#pragma pack(push, 1)
struct TelemetryFrame {
    uint8_t sync1 = TELEMETRY_SYNC1;
    uint8_t sync2 = TELEMETRY_SYNC2;
    uint16_t seq;
    uint32_t timestamp_ms;
    float altitude_m;
    float battery_v;
    float cpu_temp_c;
    uint8_t gps_fix;
    uint8_t flags; // 0=armed, 1=streaming, 2=low_bat
    uint16_t crc16;
};
#pragma pack(pop)

inline uint16_t calc_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
    return crc;
}
