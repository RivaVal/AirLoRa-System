

////### 📁 Файл 2: `GY87_Handler.h` — Обновлённый заголовок 
//с поддержкой фильтрации
//

/**
* @file GY87_Handler.h
* @brief Неблокирующий обработчик сенсора GY-87 с комплементарным фильтром для углов Эйлера
* @version 5.0.0 - Полная поддержка стабилизации полёта
*
* @details
* 🎯 КЛЮЧЕВЫЕ УЛУЧШЕНИЯ:
* - Комплементарный фильтр для вычисления углов Эйлера (roll, pitch, yaw)
* - Автоматическая калибровка гироскопа при старте
* - Защита от дрейфа гироскопа и шума акселерометра
* - Неблокирующая архитектура на основе конечного автомата
* - Подробная отладка через ESP_LOG (ESP-IDF 5.0+ совместимость)
*
* 📊 АРХИТЕКТУРА ФИЛЬТРАЦИИ:
*   Угол = α * (интеграл гироскопа) + (1-α) * (угол из акселерометра)
*   где α = 0.982 для 100 Гц обновления (оптимальный баланс)
*
* ⚙️ ИНТЕГРАЦИЯ С СИСТЕМОЙ УПРАВЛЕНИЯ:
*   - Крен (roll) → управление элеронами (серво #0, #1)
*   - Тангаж (pitch) → управление рулями высоты (серво #2, #3)
*   - Рыскание (yaw) → управление рулём направления (серво #4)
*
* @author Embedded Systems Team
* @version 5.0.0
* @date 2026-02-06
*/
//#ifndef GY87_HANDLER_H
//#define GY87_HANDLER_H

/**
 * @file GY87_Handler.h
 * @brief Заголовочный файл для обработчика GY-87 (MPU6050) через I2CMasterController.
 * @version 1.0.0
 * @date 2026-02-12
 *
 * @details
 * - Использует I2CMasterController для взаимодействия по шине I2C.
 * - Инициализирует и считывает данные с MPU6050.
 * - Пока НЕ включает HMC5883L (требует дальнейшей адаптации).
 */

#pragma once

#ifndef GY87_HANDLER_H
#define GY87_HANDLER_H

#include <Arduino.h>
#include "esp_log.h"
#include "CommonTypes.h" // Убедитесь, что struct SensorData определён здесь
// Forward declaration
// class I2CMasterController;
#include "I2C_Master.h"  // Полное включение вместо forward declaration

/**
 * @file GY87_Handler.h
 * @brief Заголовочный файл для обработчика GY-87 (MPU6050) через I2CMasterController.
 * @version 1.0.0
 * @date 2026-02-12
 *
 * @details
 * - Использует I2CMasterController для взаимодействия по шине I2C.
 * - Инициализирует и считывает данные с MPU6050.
 * - Пока НЕ включает HMC5883L (требует дальнейшей адаптации).
 */
class GY87Handler {
public:
    /**
     * @brief Конструктор.
     */
    GY87Handler();

    /**
     * @brief Инициализирует MPU6050 через предоставленный I2C Manager.
     * @param i2c_manager Ссылка на инициализированный I2CMasterController.
     * @return true, если инициализация прошла успешно, иначе false.
     */
    bool begin(I2CMasterController& i2c_manager);

    /**
     * @brief Проверяет, была ли выполнена инициализация.
     * @return true, если инициализирован, иначе false.
     */
    bool isInitialized() const;

    /**
     * @brief Проверяет, действительны ли последние полученные данные.
     * @return true, если данные валидны, иначе false.
     */
    bool isDataValid() const;

    /**
     * @brief Обновляет данные с MPU6050.
     * @return true, если обновление прошло успешно, иначе false.
     */
    bool updateSensors();

    /**
     * @brief Возвращает ссылку на последнюю структуру SensorData.
     * @return const SensorData&.
     */
    const SensorData& getData() const;

    // Старые геттеры (опционально, можно удалить, если не нужны)
    int16_t getAccelX() const;
    int16_t getAccelY() const;
    int16_t getAccelZ() const;
    int16_t getGyroX() const;
    int16_t getGyroY() const;
    int16_t getGyroZ() const;
    int16_t getTemperature() const;

    float getAccelX_mss() const;
    float getAccelY_mss() const;
    float getAccelZ_mss() const;
    float getGyroX_rads() const;
    float getGyroY_rads() const;
    float getGyroZ_rads() const;
    float getTemperature_C() const;

private:
    I2CMasterController* _i2cManager;
    bool _initialized;
    SensorData _sensorData; // Объект для хранения углов и т.д.

    // Сырые данные (опционально, можно убрать)
    int16_t _accelX, _accelY, _accelZ;
    int16_t _gyroX, _gyroY, _gyroZ;
    int16_t _temperature;

        // Поля для калибровки гироскопа
    bool _gyroCalibrated;
    uint16_t _calibrationCount;
    float _gyroOffsetX;
    float _gyroOffsetY;
    float _gyroOffsetZ;
    uint32_t _lastUpdate;
};

#endif // GY87_HANDLER_H