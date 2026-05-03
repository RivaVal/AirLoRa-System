

/**
 * @file TelemetryBridge.h
 * @brief Интерфейс моста телеметрии ESP32-S3 ↔ RPi Zero 2W
 */
// Решение строго опирается на ваши структуры (DataComSet_t, 
//  SensorData, BatteryStatus_t), использует пины UART2 
//  из Config.h и гарантирует неблокирующую работу 
//  в реальном времени.
//  📄 ФАЙЛ 1: TelemetryBridge.h (Создайте новый)
//  📍 Позиция: В папке проекта рядом с .ino
//  🧩 Назначение: Объявление упакованной структуры телеметрии и интерфейса моста
//  💻 Код:

#pragma once
#ifndef TELEMETRY_BRIDGE_H
#define TELEMETRY_BRIDGE_H

#include <Arduino.h>
#include "CommonTypes.h" // DataComSet_t, BatteryStatus_t, SensorData, TimerMillis
#include "BatteryMonitor.h"
#include "Config.h"
            // #include "CommonTypes.h" // Для TimerMillis

/**
 * @brief Бинарный пакет телеметрии для UART2 → RPi Zero 2W
 * @brief Бинарный пакет телеметрии (фиксированный размер 42 байта)
 * @details Структура упакована __attribute__((packed)) для гарантии
 *          точного размера и отсутствия выравнивания компилятором.
 *          Совместима с форматом unpack '<HIBBH B 4f 4f 2f f B B b B' в Python.
 * @details Структура упакована __attribute__((packed)) для исключения выравнивания компилятором.
 *          Поля строго соответствуют требованиям ТЗ: команды + IMU + батарея + статус.
 */
class TelemetryBridge {
public:
    /**
     * @brief Инициализация UART2 для связи с RPi
     * @return true при успехе
     */
    static bool begin();
    
    /**
     * @brief Обновление и отправка телеметрии (вызывать в loop())
     * @param timer Таймер контроля частоты (20-50 мс)
     * @param com Данные управления из LoRa
     * @param imu Данные стабилизации
     * @param bat Статус батареи
     */
    static void update(TimerMillis* timer, 
                      const DataComSet_t& com, 
                      const SensorData& imu, 
                      const BatteryStatus_t& bat);
    
    /**
     * @brief Приём команд от RPi (обратный канал)
     * @param buffer Буфер для данных
     * @param maxLength Максимальный размер
     * @return Количество принятых байт
     */
    static int receiveCommand(uint8_t* buffer, size_t maxLength);
    
    /**
     * @brief Печать статистики работы
     */
    static void printStats();
    
private:
    /**
     * @brief Внутренняя отправка пакета (неблокирующая)
     */
    static bool send(const TelemetryPacket_t& packet);
};

#endif // TELEMETRY_BRIDGE_H
