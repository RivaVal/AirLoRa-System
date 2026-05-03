

// ### 📁 ФАЙЛ 1: `TelemetryPacket.h` / `TelemetrySender.cpp` (ESP32-S3)
// **📍 Позиция:** В проекте ESP32, рядом с `LoRaCommunicator` и `FlightStabilizer`
// **🧩 Структура:** Упаковка `DataComSet_t` + расчётные данные в бинарный пакет
// **💻 Код:**
//
// ### Шаг 1: Создать/обновить `TelemetryPacket.h`
//
#pragma once
#ifndef TELEMETRY_PACKET_H
#define TELEMETRY_PACKET_H

#include <cstdint>
#include "CommonTypes.h"
#include <Arduino.h>
// #include <cstdint>
// #include "CommonTypes.h" // DataComSet_t

#pragma pack(push, 1)  // Выравнивание 1 байт для бинарной совместимости
#pragma once

/**
 * @brief Бинарный пакет телеметрии для передачи по UART
 * @details Размер: 44 байта (без учёта framing + CRC).
 *          Упакован __attribute__((packed)) для гарантии отсутствия выравнивания.
 *          Передаётся с частотой 20 Гц → ~880 байт/сек (тривиально для 115200 бод).
 * @file TelemetryPacket.h
 * @brief Бинарная структура телеметрии для передачи через UART2 на RPi Zero 2W
 * @version 1.0.0
 * @date 2026
 * 
 * @details
 * • Размер пакета: 44 байта (фиксированный, для детерминированной передачи)
 * • Фрейминг: 0xAA 0x55 [PAYLOAD] CRC8 0xCC 0x33
 * • Частота: 20-50 мс (20-50 Гц)
 * • Скорость UART: 921600 бод с аппаратным Flow Control
 */


struct TelemetryPacket_t {
    // === ФРЕЙМИНГ ===
    uint8_t  header1;          // 0xAA — начало пакета
    uint8_t  header2;          // 0x55 — подтверждение начала
    
    // === ДАННЫЕ УПРАВЛЕНИЯ (из DataComSet_t) ===
    uint32_t timestamp;        // Время отправки (millis)
    uint16_t packet_id;        // ID пакета для отслеживания потерь
    uint8_t  com_up;           // Команда тангажа (0-255)
    uint8_t  com_left;         // Команда крена (0-255)
    uint16_t com_throttle;     // Тяга (1000-2000)
    uint8_t  com_flags;        // Битовая маска команд
    
    // === ДАННЫЕ СТАБИЛИЗАЦИИ (из FlightStabilizer/MPU9250) ===
    float    roll;             // Крен (-90°..+90°)
    float    pitch;            // Тангаж (-90°..+90°)
    float    yaw;              // Рыскание (0°..360°)
    float    altitude;         // Высота (м)
    float    speed;            // Скорость (м/с) — 0 если нет GPS
    double   latitude;         // Широта (градусы) — 0 если нет фиксации
    double   longitude;        // Долгота (градусы)
    
    // === СОСТОЯНИЕ БАТАРЕИ ===
    float    bat_voltage;      // Напряжение АКБ (В)
    uint8_t  bat_percent;      // Заряд (0-100%)
    
    // === СТАТУС СИСТЕМЫ ===
    uint8_t  flight_mode;      // 1=MANUAL, 2=STAB, 3=RTL
    int8_t   rssi;             // Уровень сигнала LoRa (-120..0 dBm)
    
    // === ФРЕЙМИНГ ===
    uint8_t  footer1;          // 0xCC — конец данных
    uint8_t  footer2;          // 0x33 — подтверждение конца
    uint8_t  crc8;             // CRC8 полином 0x07 (исключая header+footer+crc)
};

#pragma pack(pop)

// Константы размера
constexpr size_t TELEMETRY_PAYLOAD_SIZE = sizeof(TelemetryPacket_t) - 4;  // Без framing+CRC
constexpr size_t TELEMETRY_FULL_SIZE = sizeof(TelemetryPacket_t);          // 44 байта

// Прототип функции расчёта CRC8 (реализация в utils.cpp)
uint8_t calculateCRC8(const uint8_t* data, size_t length);

#endif // TELEMETRY_PACKET_H

/**
 * @class TelemetrySender
 * @brief Неблокирующий отправитель телеметрии в UART
 *
class TelemetrySender {
public:
    static void begin(HardwareSerial& port, uint32_t baud = 115200);
    static bool send(const TelemetryPacket_t& pkt);
    static uint8_t calcCRC8(const uint8_t* data, size_t len);
private:
    static HardwareSerial* _uart;
    static constexpr uint8_t SYNC1 = 0xAA;
    static constexpr uint8_t SYNC2 = 0x55;
    static constexpr uint8_t END1  = 0xCC;
    static constexpr uint8_t END2  = 0x33;
};
*/