#include <cstdint>



//### 3. Создайте файл `BatteryMonitor.h`:
//
//
/**
* @file BatteryMonitor.h
* @brief Мониторинг напряжения аккумулятора с поддержкой 3S-12S LiPo
* @version 2.0.0 - Добавлена таблица конфигураций АКБ
* @date 2026
* 
* @details
* - Поддержка конфигураций от 3S до 12S LiPo/Li-Ion
* - Автоматический расчет порогов на основе количества ячеек
* - Периодическая проверка в loop()
* - Аварийные действия при критическом разряде
* - Подробная отладка через ESP_LOG
* 
* @note Требуется внешний делитель напряжения для подключения к GPIO34!
*/
/**
* @file BatteryMonitor.h
* @brief Мониторинг напряжения аккумулятора с поддержкой 3S-12S LiPo
* @version 2.0.1 - Исправлен конфликт имён с Config.h
* @date 2026
*
* @details
* - Поддержка конфигураций от 3S до 12S LiPo/Li-Ion
* - Автоматический расчет порогов на основе количества ячеек
* - Периодическая проверка в loop()
* - Аварийные действия при критическом разряде
* - Подробная отладка через ESP_LOG
*
* @note Требуется внешний делитель напряжения для подключения к GPIO34!
*/
/**
 * @file BatteryMonitor.h
 * @brief Мониторинг напряжения аккумулятора с поддержкой 3S-12S LiPo
 * @version 2.0.2 - Исправлен конфликт имён, обновлён ADC драйвер
 * @date 2026
 * 
 * @details
 * - Поддержка конфигураций от 3S до 12S LiPo/Li-Ion
 * - Новый ADC драйвер esp_adc/adc_oneshot.h (ESP-IDF 5.0+)
 * - Автоматический расчет порогов на основе количества ячеек
 * - Периодическая проверка в loop()
 * - Аварийные действия при критическом разряде
 * - Подробная отладка через ESP_LOG
 * 
 * @note Требуется внешний делитель напряжения для подключения к GPIO34!
 */
/**
 * @file BatteryMonitor.h
 * @brief Мониторинг напряжения аккумулятора с поддержкой 3S-12S LiPo
 * @version 2.0.5 - Полная совместимость с ESP-IDF 5.0+
 * @date 2026
 */
#pragma once
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
// ✅ НОВЫЙ ADC драйвер (ESP-IDF 5.0+)
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_log.h>
#include "Config.h"
#include "CommonTypes.h"


/**
* @class BatteryMonitor
* @brief Класс мониторинга состояния аккумуляторной батареи
* 
* @details Реализует:
* - Чтение ADC с усреднением (16 выборок)
* - Преобразование в напряжение батареи с учетом делителя
* - Расчет процента заряда по линейной аппроксимации
* - Определение статуса (OK/Low/Critical)
* - Периодическую проверку без блокировки
*/
class BatteryMonitor {
public:
    /**
     * @brief Конструктор монитора батареи
     */
    BatteryMonitor();
    ~BatteryMonitor();  // ✅ Деструктор объявлен
    
    /**
    * @brief Инициализация монитора батареи
    * @param cellCount Количество ячеек (3-12)
    * @return true при успешной инициализации
    */
    //   bool begin(uint8_t cellCount = BatteryMonitor::DEFAULT_CELL_COUNT);
    // bool begin(uint8_t cellCount = DEFAULT_CELL_COUNT);
    bool begin(uint8_t cellCount = Config::Battery::DEFAULT_CELL_COUNT);
    
    /**
    * @brief Проверка напряжения батареи (немедленная)
    * @return Структура с текущим состоянием батареи
    */
    BatteryStatus_t checkVoltage();
    
    /**
    * @brief Периодическая проверка (вызывать в loop())
    * @note Неблокирующая функция с внутренним таймером
    */
    void update();
    
    /**
     * @brief Получение последнего статуса батареи
     * @return Копия последнего статуса
     */
    BatteryStatus_t getStatus() const { return _lastStatus; }
    
    /**
     * @brief Проверка, можно ли работать (напряжение в норме)
     * @return true если напряжение OK
     */
    bool isVoltageOk() const { return _lastStatus.isOk; }
    
    /**
     * @brief Проверка на критически низкое напряжение
     * @return true если напряжение критическое
     */
    bool isCritical() const { return _lastStatus.isCritical; }
    
    /**
    * @brief Проверка на низкое напряжение
    * @return true если напряжение низкое
    */
    bool isLow() const { return _lastStatus.isLow; }
    
    /**
    * @brief Установка конфигурации АКБ
    * @param cellCount Количество ячеек (3-12)
    * @return true если конфигурация допустима
    */
    bool setCellCount(uint8_t cellCount);
    
    /**
    * @brief Получение текущей конфигурации
    * @return Количество ячеек
    */
    uint8_t getCellCount() const { return _currentConfig.cellCount; }
    
    /**
    * @brief Печать статуса батареи в лог
    * @param verbose Уровень детализации (true = подробно)
    */
    void printStatus(bool verbose = false) const;
    
    /**
    * @brief Получение напряжения на ячейку
    * @return Напряжение на одну ячейку (В)
    */
    float getVoltagePerCell() const { return _lastStatus.voltagePerCell; }
    
    /**
    * @brief Сброс счетчиков и статистики
    */
    void reset();
    
    
private:
    BatteryStatus_t _lastStatus;
    BatteryConfig_t _currentConfig;
    bool _initialized;
    uint32_t _checkCount;
    uint32_t _criticalCount;
    
    // ✅ ДОБАВЛЕНО: Поля ADC-контекста (обязательно объявить здесь!)
    adc_oneshot_unit_handle_t _adcHandle;
    adc_cali_handle_t _adcCaliHandle;
    bool _adcCaliValid;
    
    static const char* TAG;
    
    // ✅ ДОБАВЛЕНО: Объявления всех приватных методов
    bool initADC();
    bool readADC(int &adcValue);
    
    /**
     * @brief Преобразование ADC значения в напряжение батареи
     * @param adcValue Сырое значение ADC (0-4095)
     * @return Напряжение батареи в вольтах
     */
    float adcToBatteryVoltage(int adcValue) const;  // ✅ int вместо uint16_t
    
    /**
     * @brief Расчет процента заряда
     * @param voltage Напряжение батареи
     * @return Процент заряда (0-100)
     */
    float calculatePercentage(float voltage) const;
    
    /**
    * @brief Поиск конфигурации по количеству ячеек
    * @param cellCount Количество ячеек
    * @return Указатель на конфигурацию или nullptr
    */
    const BatteryConfig_t* findConfig(uint8_t cellCount) const;
    
    /**
    * @brief Обновление статусов на основе напряжения
    * @param voltage Текущее напряжение
    */
    void updateStatus(float voltage);

    // 🔧 ДОБАВИТЬ: Кроссплатформенные хелперы для ADC (ESP32 / ESP32-S3)
    /**
     * @brief 🔑 Маппинг GPIO → ADC_UNIT
     * @details На ESP32-S3: GPIO1-10 → ADC_UNIT_1 (с калибровкой), 
     *          GPIO11-20 → ADC_UNIT_2 (только raw-чтение).
     *          ADC_UNIT_2 не поддерживает Line Fitting в ESP-IDF 5.0+
     * @param gpio Номер GPIO пина
     * @return ADC_UNIT_1 или ADC_UNIT_2
     */
    static adc_unit_t _gpioToAdcUnit(uint8_t gpio);
    
    /**
     * @brief 🔑 Маппинг GPIO → ADC_CHANNEL
     * @details Учитывает различия в маппинге каналов между ESP32 и ESP32-S3
     * @param gpio Номер GPIO пина
     * @return Номер канала ADC
     */
    static adc_channel_t _gpioToAdcChannel(uint8_t gpio);
    
    // 🔧 ДОБАВИТЬ: Отдельный тег для логирования батареи
    static const char* TAG_BAT;  ///< Тег для ESP_LOG в BatteryMonitor

};

#endif // BATTERY_MONITOR_H
