// esp32_uart_sender.ino
#include "uart_protocol.h"
#include <HardwareSerial.h>

HardwareSerial UART2(2);  // UART2: GPIO16(TX), GPIO17(RX)
uint16_t seq_counter = 0;

void setup() {
    Serial.begin(115200);  // Отладка
    UART2.begin(921600, SERIAL_8N1, 17, 16);  // 921600 бод, пины для Pi
    pinMode(4, OUTPUT);  // LED indicator
}

void send_telemetry_basic(float alt, float bat, uint8_t mode) {
    UartPacket<TelemetryBasic> pkt;

    // Заголовок
    pkt.header.sync1 = UART_SYNC_1;
    pkt.header.sync2 = UART_SYNC_2;
    pkt.header.msg_type = MSG_TELEMETRY_BASIC;
    pkt.header.payload_len = sizeof(TelemetryBasic);
    pkt.header.seq_num = seq_counter++;

    // Полезная нагрузка
    pkt.payload.altitude = alt;
    pkt.payload.battery_voltage = bat;
    pkt.payload.flight_mode = mode;
    pkt.payload.reserved = 0;

    // CRC (считаем по заголовку + полезной нагрузке)
    pkt.crc16 = crc16_ccitt((uint8_t*)&pkt.header, sizeof(UartHeader) + sizeof(TelemetryBasic));

    // Отправка
    UART2.write((uint8_t*)&pkt, sizeof(pkt));
    UART2.flush();

    // Индикация
    digitalWrite(4, HIGH); delay(10); digitalWrite(4, LOW);
}

void loop() {
    // Эмуляция данных (замени на реальные сенсоры)
    float alt = 12.5 + 0.1 * millis() / 1000.0;  // Плавный рост
    float bat = 3.85 - 0.001 * millis() / 1000.0; // Плавный разряд
    uint8_t mode = 0;  // STABILIZE

    send_telemetry_basic(alt, bat, mode);

    // Heartbeat каждые 500 мс
    static uint32_t last_hb = 0;
    if (millis() - last_hb >= 500) {
        // Отправка пустого heartbeat (payload_len=0)
        UartHeader hb = {UART_SYNC_1, UART_SYNC_2, MSG_HEARTBEAT, 0, seq_counter++};
        uint16_t crc = crc16_ccitt((uint8_t*)&hb, sizeof(hb));
        UART2.write((uint8_t*)&hb, sizeof(hb));
        UART2.write((uint8_t*)&crc, sizeof(crc));
        last_hb = millis();
    }

    delay(100);  // ~10 Гц телеметрия
}
