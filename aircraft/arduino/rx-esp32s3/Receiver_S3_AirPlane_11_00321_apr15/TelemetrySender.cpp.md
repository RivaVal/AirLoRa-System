

// cpp
// TelemetrySender.cpp
#include "TelemetryPacket.h"
HardwareSerial* TelemetrySender::_uart = nullptr;

void TelemetrySender::begin(HardwareSerial& port, uint32_t baud) {
    _uart = &port;
    _uart->begin(baud, SERIAL_8N1, 5, 6); // GPIO5(TX), GPIO6(RX) на ESP32-S3
    _uart->setRxBufferSize(128);
    _uart->setTxBufferSize(256);
}

uint8_t TelemetrySender::calcCRC8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
    return crc;
}

bool TelemetrySender::send(const TelemetryPacket_t& pkt) {
    if (!_uart || _uart->availableForWrite() < sizeof(TelemetryPacket_t) + 6) return false;

    // 1. Заголовок синхронизации
    _uart->write(SYNC1); _uart->write(SYNC2);
    // 2. Тело пакета (без CRC)
    _uart->write((const uint8_t*)&pkt, sizeof(TelemetryPacket_t) - 1);
    // 3. Расчёт и запись CRC
    uint8_t crc = calcCRC8((const uint8_t*)&pkt, sizeof(TelemetryPacket_t) - 1);
    _uart->write(crc);
    // 4. Завершающий маркер
    _uart->write(END1); _uart->write(END2);
    return true;
}
