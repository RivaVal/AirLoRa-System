#include "telemetry_protocol.h"
#include <HardwareSerial>
//📤 Отправитель (ESP32, ESP-IDF / Arduino)

HardwareSerial TelemetrySerial(2); // UART2: TX=17, RX=16
uint16_t seq = 0;

void sendTelemetry(float alt, float bat, float temp, bool armed, bool gps_ok) {
    TelemetryFrame frame;
    frame.seq = seq++;
    frame.timestamp_ms = millis();
    frame.altitude_m = alt;
    frame.battery_v = bat;
    frame.cpu_temp_c = temp;
    frame.gps_fix = gps_ok ? 1 : 0;
    frame.flags = (armed ? 1 : 0) | (1 << 1); // streaming flag
    
    // CRC считается по всему пакету БЕЗ поля crc16
    frame.crc16 = calc_crc16((uint8_t*)&frame, sizeof(TelemetryFrame) - sizeof(frame.crc16));
    
    TelemetrySerial.write((uint8_t*)&frame, sizeof(frame));
}
// Вызывай 10-50 Гц в задаче или таймере
