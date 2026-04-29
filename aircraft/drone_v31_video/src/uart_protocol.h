// uart_protocol.h
#pragma once
#include <stdint.h>

// Синхробайты
#define UART_SYNC_1 0xAA
#define UART_SYNC_2 0x55
#define UART_MAX_PAYLOAD 255

// Типы сообщений
enum MessageType : uint8_t {
    MSG_TELEMETRY_BASIC = 0x01,
    MSG_TELEMETRY_GPS = 0x02,
    MSG_TELEMETRY_EXTENDED = 0x03,
    MSG_CMD_VIDEO_REC = 0x10,
    MSG_CMD_MODE_CHANGE = 0x11,
    MSG_HEARTBEAT = 0xFF
};

// Базовый заголовок пакета (6 байт)
struct __attribute__((packed)) UartHeader {
    uint8_t sync1;          // 0xAA
    uint8_t sync2;          // 0x55
    uint8_t msg_type;       // MessageType
    uint8_t payload_len;    // 0..255
    uint16_t seq_num;       // Sequence number (little-endian)
};

// Полезная нагрузка: базовая телеметрия (8 байт)
struct __attribute__((packed)) TelemetryBasic {
    float altitude;         // м, 4 байта, float
    float battery_voltage;  // В, 4 байта, float
    uint8_t flight_mode;    // 0=STAB, 1=LOITER, 2=AUTO...
    uint8_t reserved;       // Выравнивание
};

// Полный пакет (заголовок + данные + CRC)
template<typename PayloadT>
struct UartPacket {
    UartHeader header;
    PayloadT payload;
    uint16_t crc16;

    static constexpr size_t SIZE = sizeof(UartHeader) + sizeof(PayloadT) + sizeof(uint16_t);
};

// CRC16-CCITT (полином 0x1021, инициализация 0xFFFF)
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}
