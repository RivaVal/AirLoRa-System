

/**
 * @file Unified_LEDC_Controller.h
 * @brief Контроллер управления моторами через встроенный LEDC ESP32
 * 
 * АРХИТЕКТУРА:
 * - Управление 2 моторами через аппаратный ШИМ LEDC
 * - Пины: Мотор #0 → GPIO32, Мотор #1 → GPIO33
 * - Частота: 1000 Гц (оптимально для моторов)
 * - Разрешение: 10 бит (1024 уровня мощности)
 * 
 * БЕЗОПАСНОСТЬ:
 * - Максимальная мощность в тестах ограничена 35%
 * - Все методы включают проверку границ
 * - Отладка через ESP_LOG для мониторинга состояния
 * 
 * @author Embedded Systems Team
 * @version 1.1.0 (исправленная)
 * @date 2026-02-13
 */
#ifndef UNIFIED_LEDC_CONTROLLER_H
#define UNIFIED_LEDC_CONTROLLER_H

#include <Arduino.h>
#include "esp_err.h"
#include "driver/ledc.h"
#include "Config.h"


class UnifiedLEDCController {
private:
    bool _initialized = false;
    const char* _lastError = "No error";
    
    // Настройки каналов для моторов (фиксированные пины из Config.h)
    const ledc_channel_t _motorChannels[2] = {
        LEDC_CHANNEL_0,  // Мотор 0 → GPIO32
        LEDC_CHANNEL_1   // Мотор 1 → GPIO33
    };
    
    /**
     * @brief Настройка таймера и каналов LEDC
     * @return true при успехе
     */
    bool setupLEDC();

public:
    UnifiedLEDCController();
    
    /**
     * @brief Инициализация контроллера моторов
     * @return true при успешной инициализации
     */
    bool begin();
    
    /**
     * @brief Установка мощности мотора (0-100%)
     * @param motorIndex Индекс мотора (0 или 1)
     * @param powerPercent Мощность в процентах (0.0 - 100.0)
     * @return true при успехе
     */
    bool setMotorPower(uint8_t motorIndex, float powerPercent);
    
    /**
     * @brief Безопасный тест моторов с ограничением мощности
     * @param durationMs Длительность теста в миллисекундах
     * @return true при успешном завершении
     */
    bool runMotorTest(uint32_t durationMs = 2000);
    
    /**
     * @brief Остановка всех моторов (0% мощности)
     */
    void stopAllMotors();
    
    /**
     * @brief Получение последней ошибки
     * @return Строка с описанием ошибки
     */
    const char* getLastError() const { return _lastError; }
    
    /**
     * @brief Проверка статуса инициализации
     * @return true если контроллер инициализирован
     */
    bool isInitialized() const { return _initialized; }
};

#endif // UNIFIED_LEDC_CONTROLLER_H
