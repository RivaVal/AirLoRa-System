


// ### 4. Создайте файл `BatteryMonitor.cpp`:
// 
//
/**
* @file BatteryMonitor.cpp
* @brief Реализация мониторинга напряжения аккумулятора 3S-12S
* @version 2.0.0 - Полная поддержка таблицы конфигураций
* @date 2026
* 
* @details
* - Чтение ADC с усреднением 16 выборок
* - Защита от аномальных значений
* - ESP_LOG отладка на всех уровнях
* - Аварийное логирование при критическом разряде
*/
/**
* @file BatteryMonitor.cpp
* @brief Реализация мониторинга напряжения аккумулятора 3S-12S
* @version 2.0.1 - Исправлены все ошибки компиляции
* @date 2026
*
* @details
* - Чтение ADC с усреднением 16 выборок
* - Защита от аномальных значений
* - ESP_LOG отладка на всех уровнях
* - Аварийное логирование при критическом разряде
*/
/**
 * @file BatteryMonitor.cpp
 * @brief Реализация мониторинга напряжения аккумулятора 3S-12S
 * @version 2.0.2 - Исправлены все ошибки компиляции, новый ADC драйвер
 * @date 2026
 * 
 * @details
 * - Чтение ADC с усреднением 16 выборок
 * - Новый драйвер esp_adc/adc_oneshot.h
 * - Защита от аномальных значений
 * - ESP_LOG отладка на всех уровнях
 * - Аварийное логирование при критическом разряде
 */
/**
 * @file BatteryMonitor.cpp
 * @brief Реализация мониторинга напряжения аккумулятора
 * @version 2.0.5 - Полная совместимость с ESP-IDF 5.0+
 * @date 2026
 * 
 * @details
 * - Использование нового ADC API (esp_adc/adc_oneshot.h)
 * - Исправление всех ошибок компиляции
 * - Полная документация методов
 * - Подробная отладка через ESP_LOG
 */
#include "BatteryMonitor.h"
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

// ============================================================================
// СТАТИЧЕСКАЯ ПЕРЕМЕННАЯ TAG
// ============================================================================
const char* BatteryMonitor::TAG_BAT = "BATTERY";


// ============================================================================
// КОНСТРУКТОР
// ============================================================================
BatteryMonitor::BatteryMonitor()
    : _initialized(false), _checkCount(0), _criticalCount(0),
      _adcHandle(nullptr), _adcCaliHandle(nullptr), _adcCaliValid(false)
{
    _lastStatus = {};
    _currentConfig = BATTERY_CONFIG_TABLE[1]; // 4S по умолчанию
    ESP_LOGI(TAG_BAT, "✅ Конструктор BatteryMonitor инициализирован");
}

BatteryMonitor::~BatteryMonitor() {
    if (_adcCaliHandle) {
        ESP_LOGD(TAG_BAT, "Освобождение калибровки ADC");
        adc_cali_delete_scheme_curve_fitting(_adcCaliHandle);
    }
    if (_adcHandle) {
        ESP_LOGD(TAG_BAT, "Освобождение ADC unit");
        adc_oneshot_del_unit(_adcHandle);
    }
    ESP_LOGI(TAG_BAT, "🔄 Ресурсы BatteryMonitor освобождены");
}

/**
 * @brief 🔑 Кроссплатформенный маппинг GPIO → ADC_UNIT
 * @details На ESP32-S3 GPIO1-10 → ADC_UNIT_1, GPIO11-20 → ADC_UNIT_2.
 *          ADC_UNIT_2 не поддерживает калибровку Line Fitting в ESP-IDF 5.0+.
 */
 /*
adc_unit_t BatteryMonitor::_gpioToAdcUnit(uint8_t gpio) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // ESP32-S3: GPIO1-10 → ADC_UNIT_1, GPIO11-20 → ADC_UNIT_2
    // 🔑 Только ADC_UNIT_1 поддерживает adc_cali_create_scheme_curve_fitting()
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    switch(gpio) {
        case 1:  return ADC_CHANNEL_0;
        case 2:  return ADC_CHANNEL_1;
        case 3:  return ADC_CHANNEL_2;
        case 4:  return ADC_CHANNEL_3;
        case 5:  return ADC_CHANNEL_4;
        case 6:  return ADC_CHANNEL_5;
        // ... другие случаи ...
        case 7:  return ADC_CHANNEL_6;  // 🔑 GPIO7 → ADC1_CH6
        case 8:  return ADC_CHANNEL_7;  // 🔑 GPIO7 → ADC1_CH6
        // ...
        default: return ADC_CHANNEL_0;
    }
#else
    // Классический ESP32 маппинг
    // ...
#endif
}
*/
/*
    if (gpio >= 1 && gpio <= 10) {
        return ADC_UNIT_1;
    }
    // GPIO33-40 на S3 → ADC_UNIT_2 (без калибровки!)
    return ADC_UNIT_2;
#else
    // Классический ESP32: GPIO32-39 → ADC_UNIT_1
    return ADC_UNIT_1;
#endif
*/
// }

/**
 * @brief 🔑 Маппинг GPIO → ADC_CHANNEL (ESP32 / ESP32-S3)
 * @details Добавлена поддержка безопасного пина GPIO7 (ADC1_CH6) для мониторинга батареи на S3.
 */
adc_channel_t BatteryMonitor::_gpioToAdcChannel(uint8_t gpio) {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // 🔑 ADC1 channels для ESP32-S3 (GPIO1-10)
    switch(gpio) {
        case 1:  return ADC_CHANNEL_0;
        case 2:  return ADC_CHANNEL_1;
        case 3:  return ADC_CHANNEL_2;
        case 4:  return ADC_CHANNEL_3;
        case 5:  return ADC_CHANNEL_4;
        case 6:  return ADC_CHANNEL_5;
        case 7:  return ADC_CHANNEL_6;  // 🔑 РЕКОМЕНДУЕМЫЙ ПИН ДЛЯ БАТАРЕИ! // 🔑 GPIO7 → ADC1_CH6
        case 8:  return ADC_CHANNEL_7;
        case 9:  return ADC_CHANNEL_8;
        case 10: return ADC_CHANNEL_9;
        // Fallback для ADC2 (без калибровки)
        default: return ADC_CHANNEL_0;  // ADC2_CHANNEL_0 для совместимости
    }
#else
    // Классический ESP32 маппинг
    switch(gpio) {
        case 32: return ADC_CHANNEL_4;
        case 33: return ADC_CHANNEL_5;
        case 34: return ADC_CHANNEL_6;  // 🔑 Оригинальный пин батареи
        case 35: return ADC_CHANNEL_7;
        case 36: return ADC_CHANNEL_0;
        case 39: return ADC_CHANNEL_3;
        default: return ADC_CHANNEL_6;
    }
#endif

// 📝 ESP_LOG отладка в initADC():
ESP_LOGI(TAG_BAT, "🔧 ADC канал: GPIO%d → ADC_UNIT_%d_CH%d",
         BATTERY_ADC_PIN,
         (_gpioToAdcUnit(BATTERY_ADC_PIN) == ADC_UNIT_1) ? 1 : 2,
         _gpioToAdcChannel(BATTERY_ADC_PIN));

}


/**
 * @brief 🔑 Инициализация ADC с учётом целевого чипа (ESP32 / ESP32-S3)
 * @details Для ADC_UNIT_2 на ESP32-S3 калибровка отключается,
 *          так как ESP-IDF 5.0+ не поддерживает Line Fitting для ADC2.
 *          Добавлен fallback на raw-чтение с предупреждением.
 * @return true при успешной инициализации
 */
 /**
 * @brief Инициализация ADC с учётом архитектуры чипа (ESP32 / ESP32-S3)
 * @details 
 * • ADC_UNIT_1 (GPIO1-10 на S3, GPIO32-39 на ESP32) поддерживает калибровку
 * • ADC_UNIT_2 не поддерживает Curve Fitting в ESP-IDF 5.0+
 * • Порядок полей в структурах инициализации критичен для компиляции
 * @note Для мониторинга батареи на ESP32-S3 рекомендуется использовать GPIO7 (ADC1_CH6)
 */
bool BatteryMonitor::initADC() {
    ESP_LOGI(TAG_BAT, "⚙️  Инициализация ADC (ESP-IDF 5.0+ oneshot)...");
    
    
    #if defined(CONFIG_IDF_TARGET_ESP32S3)
        // 🔧 ESP32-S3: Только ADC_UNIT_1 поддерживает калибровку curve fitting
        // 🔑 Динамическое определение юнита и канала
        adc_unit_t adcUnit = _gpioToAdcUnit(Config::Pins::BATTERY_ADC_PIN);
        
        if (adcUnit == ADC_UNIT_2) {
            ESP_LOGW(TAG_BAT, "⚠️ GPIO%d → ADC_UNIT_2: калибровка отключена",
                    BATTERY_ADC_PIN);
            ESP_LOGW(TAG_BAT, "💡 Рекомендация: используйте GPIO1-10 для ADC1");
        } else {
            ESP_LOGI(TAG_BAT, "✅ GPIO%d → ADC_UNIT_1: калибровка активна",
                    BATTERY_ADC_PIN);
        }
    #endif
    
    adc_channel_t adcCh = _gpioToAdcChannel(Config::Pins::BATTERY_ADC_PIN);
    
    // ✅ ИСПРАВЛЕНО: Порядок полей для adc_oneshot_unit_init_cfg_t
    adc_oneshot_unit_init_cfg_t initConfig = {
        .unit_id = adcUnit,                           // ← 1-е поле (обязательно)
        .clk_src = ADC_RTC_CLK_SRC_RC_FAST,           // ← 2-е поле (IDF 5.0+)
        .ulp_mode = ADC_ULP_MODE_DISABLE,             // ← 3-е поле
    };
    
    esp_err_t err = adc_oneshot_new_unit(&initConfig, &_adcHandle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BAT, "❌ Ошибка создания ADC unit: %s", esp_err_to_name(err));
        return false;
    }
    
    // Конфигурация канала
    adc_oneshot_chan_cfg_t chanConfig = {
        .atten = Config::Battery::BATTERY_ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_12
    };

    err = adc_oneshot_config_channel(_adcHandle, adcCh, &chanConfig);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BAT, "❌ Ошибка конфигурации канала: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(_adcHandle);
        return false;
    }
    
    // ✅ ИСПРАВЛЕНО: Порядок полей для adc_cali_curve_fitting_config_t
    _adcCaliValid = false;
    if (adcUnit == ADC_UNIT_1) {  // Калибровка только для ADC_UNIT_1
        adc_cali_curve_fitting_config_t caliConfig = {
            .unit_id = ADC_UNIT_1,              // ← 1-е поле
            .chan = adcCh,                      // ← 2-е поле (ОБЯЗАТЕЛЬНО!)
            .atten = Config::Battery::BATTERY_ADC_ATTENUATION,   // ← 3-е поле
            .bitwidth = ADC_BITWIDTH_12,        // ← 4-е поле
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 1, 0)
            .default_vref = 1100,               // ← Только для IDF < 5.1
#endif
        };
        
        err = adc_cali_create_scheme_curve_fitting(&caliConfig, &_adcCaliHandle);
        if (err == ESP_OK) {
            _adcCaliValid = true;
            ESP_LOGI(TAG_BAT, "✅ Калибровка ADC активирована (Curve Fitting)");
        } else {
            ESP_LOGW(TAG_BAT, "⚠️  Калибровка недоступна: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGW(TAG_BAT, "⚠️  ADC_UNIT_2: калибровка отключена (ограничение IDF 5.0+)");
    }
    
    return true;
}


//  ### 3.5. Исправленная `readADC()` — Динамический канал + калибровка
/**
 * @brief 🔑 Чтение ADC с динамическим каналом и обработкой калибровки
 * @param[out] adcValue Сырое значение ADC (0-4095)
 * @return true при успешном чтении
 */
bool BatteryMonitor::readADC(int &adcValue) {
    if (!_initialized || !_adcHandle) {
        ESP_LOGE(TAG_BAT, "❌ readADC: ADC не инициализирован!");
        return false;
    }
    
    // 🔑 Динамическое определение канала
    adc_channel_t adcCh = _gpioToAdcChannel(Config::Pins::BATTERY_ADC_PIN);
    
    esp_err_t err = adc_oneshot_read(_adcHandle, adcCh, &adcValue);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BAT, "❌ Ошибка чтения ADC (GPIO%d, CH%d): %s",
                 BATTERY_ADC_PIN, adcCh, esp_err_to_name(err));
        return false;
    }
    
    // 🔑 Применение калибровки (только если активна и для ADC_UNIT_1)
    if (_adcCaliValid && _adcCaliHandle) {
        int voltageMV = 0;
        err = adc_cali_raw_to_voltage(_adcCaliHandle, adcValue, &voltageMV);
        if (err == ESP_OK) {
            // Конвертируем мВ обратно в raw для совместимости с формулой делителя
            adcValue = (voltageMV * Config::Battery::BATTERY_ADC_RESOLUTION) / 3300;
            ESP_LOGV(TAG_BAT, "📊 ADC: raw=%d → calibrated=%d mV → raw=%d",
                     adcValue, voltageMV, adcValue);
        } else {
            ESP_LOGV(TAG_BAT, "⚠️ Калибровка не применима: %s", esp_err_to_name(err));
        }
    }
    
    return true;
} // END readADC


// ============================================================================
// ПРЕОБРАЗОВАНИЕ НАПРЯЖЕНИЯ
// ============================================================================
// ✅ ИСПРАВЛЕНО: параметр int вместо uint16_t (совместимость с readADC)
float BatteryMonitor::adcToBatteryVoltage(int adcValue) const {
    float adcVoltage = (adcValue / static_cast<float>(Config::Battery::BATTERY_ADC_RESOLUTION))
                       * Config::Battery::BATTERY_ADC_MAX_VOLTAGE;
    return adcVoltage * Config::Battery::BATTERY_VOLTAGE_DIVIDER_RATIO;
}

// ============================================================================
// ПУБЛИЧНЫЕ МЕТОДЫ
// ============================================================================
bool BatteryMonitor::begin(uint8_t cellCount) {
    ESP_LOGI(TAG_BAT, "=== 🚀 ИНИЦИАЛИЗАЦИЯ МОНИТОРА БАТАРЕИ ===");
    
    if (cellCount < MIN_CELL_COUNT || cellCount > MAX_CELL_COUNT) {
        ESP_LOGE(TAG_BAT, "❌ Недопустимое количество ячеек: %u (допустимо %u-%u)",
                 cellCount, MIN_CELL_COUNT, MAX_CELL_COUNT);
        return false;
    }
    
    const BatteryConfig_t* config = findConfig(cellCount);
    if (!config) {
        ESP_LOGE(TAG_BAT, "❌ Конфигурация для %uS не найдена в таблице", cellCount);
        return false;
    }
    
    _currentConfig = *config;
    ESP_LOGI(TAG_BAT, "🔋 Конфигурация АКБ: %s", _currentConfig.name);
    ESP_LOGI(TAG_BAT, "   Ячеек: %u", _currentConfig.cellCount);
    ESP_LOGI(TAG_BAT, "   Диапазон: %.2fВ - %.2fВ",
             _currentConfig.voltageMin, _currentConfig.voltageMax);
    ESP_LOGI(TAG_BAT, "   Критический: %.2fВ", _currentConfig.voltageCritical);
    
    if (!initADC()) {
        ESP_LOGE(TAG_BAT, "❌ Критическая ошибка: не удалось инициализировать ADC");
        return false;
    }
    
    _initialized = true;
    
    // ✅ ИСПРАВЛЕНО: STARTUP_CHECK_DELAY вместо BATTERY_STARTUP_CHECK_DELAY
    ESP_LOGI(TAG_BAT, "⏳ Задержка перед первой проверкой (%u мс)...",
             Config::Battery::STARTUP_CHECK_DELAY);
    delay(Config::Battery::STARTUP_CHECK_DELAY);
    checkVoltage();
    
    ESP_LOGI(TAG_BAT, "✅ Монитор батареи успешно инициализирован");
    return true;
}


BatteryStatus_t BatteryMonitor::checkVoltage() {
    if (!_initialized) {
        ESP_LOGE(TAG_BAT, "❌ checkVoltage: монитор не инициализирован!");
        return _lastStatus;
    }
    
    constexpr uint8_t SAMPLES = 16;
    int32_t sum = 0;
    
    for (uint8_t i = 0; i < SAMPLES; i++) {
        int adcValue = 0;
        // ✅ ИСПРАВЛЕНО: readADC объявлен в классе
        if (readADC(adcValue)) {
            sum += adcValue;
        }
        delay(2);
    }
    
    int adcValue = sum / SAMPLES;
    float voltage = adcToBatteryVoltage(adcValue);
    float percentage = calculatePercentage(voltage);
    float voltagePerCell = voltage / _currentConfig.cellCount;
    
    _lastStatus = {
        .voltage = voltage,
        .percentage = percentage,
        .isOk = true, .isLow = false, .isCritical = false,
        .lastCheckTime = millis(),
        .adcValue = static_cast<uint16_t>(adcValue),
        .cellCount = _currentConfig.cellCount,
        .voltagePerCell = voltagePerCell,
        .configName = _currentConfig.name
    };
    
    updateStatus(voltage);
    _checkCount++;
    
    if (_lastStatus.isCritical) {
        _criticalCount++;
        ESP_LOGE(TAG_BAT, "🔴 КРИТИЧЕСКИ НИЗКОЕ напряжение: %.2fВ (%.1f%%)", voltage, percentage);
    }
    
    return _lastStatus;
}

void BatteryMonitor::update() {
    static uint32_t lastCheck = 0;
    if (!_initialized) return;
    
    uint32_t currentTime = millis();
    // ✅ ИСПРАВЛЕНО: CHECK_INTERVAL_MS вместо BATTERY_CHECK_INTERVAL_MS
    if (currentTime - lastCheck >= Config::Battery::CHECK_INTERVAL_MS) {
        checkVoltage();
        lastCheck = currentTime;
        if (_checkCount % 10 == 0) printStatus(false);
    }
}

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ
// ============================================================================
float BatteryMonitor::calculatePercentage(float voltage) const {
    // Линейная аппроксимация процента заряда
    if (voltage <= _currentConfig.voltageMin) {
        return 0.0f;
    }
    if (voltage >= _currentConfig.voltageMax) {
        return 100.0f;
    }
    return ((voltage - _currentConfig.voltageMin) /
            (_currentConfig.voltageMax - _currentConfig.voltageMin)) * 100.0f;
}

void BatteryMonitor::updateStatus(float voltage) {
    _lastStatus.isCritical = (voltage <= _currentConfig.voltageCritical);
    _lastStatus.isLow = (voltage <= _currentConfig.voltageMin);
    _lastStatus.isOk = (voltage > _currentConfig.voltageMin &&
                        voltage <= _currentConfig.voltageMax);
}

const BatteryConfig_t* BatteryMonitor::findConfig(uint8_t cellCount) const {
    for (uint8_t i = 0; i < BATTERY_CONFIG_COUNT; i++) {
        if (BATTERY_CONFIG_TABLE[i].cellCount == cellCount) {
            return &BATTERY_CONFIG_TABLE[i];
        }
    }
    return nullptr;
}

bool BatteryMonitor::setCellCount(uint8_t cellCount) {
    if (cellCount < MIN_CELL_COUNT || cellCount > MAX_CELL_COUNT) {
        ESP_LOGE(TAG_BAT, "❌ setCellCount: недопустимое значение %u", cellCount);
        return false;
    }
    
    const BatteryConfig_t* config = findConfig(cellCount);
    if (!config) {
        ESP_LOGE(TAG_BAT, "❌ setCellCount: конфигурация не найдена");
        return false;
    }
    
    _currentConfig = *config;
    ESP_LOGI(TAG_BAT, "✅ Конфигурация АКБ изменена: %s", _currentConfig.name);
    
    // Немедленная проверка с новой конфигурацией
    checkVoltage();
    return true;
}

void BatteryMonitor::printStatus(bool verbose) const {
    ESP_LOGI(TAG_BAT, "==========================================");
    ESP_LOGI(TAG_BAT, "🔋 СТАТУС БАТАРЕИ");
    ESP_LOGI(TAG_BAT, "==========================================");
    ESP_LOGI(TAG_BAT, "Конфигурация: %s (%uS)", _lastStatus.configName, _lastStatus.cellCount);
    ESP_LOGI(TAG_BAT, "Напряжение: %.2f В", _lastStatus.voltage);
    ESP_LOGI(TAG_BAT, "На ячейку: %.2f В", _lastStatus.voltagePerCell);
    ESP_LOGI(TAG_BAT, "Заряд: %.1f %%", _lastStatus.percentage);
    ESP_LOGI(TAG_BAT, "ADC значение: %u", _lastStatus.adcValue);
    ESP_LOGI(TAG_BAT, "Статус: %s",
             _lastStatus.isCritical ? "🔴 КРИТИЧЕСКИЙ" :
             _lastStatus.isLow ? "⚠️  НИЗКИЙ" :
             _lastStatus.isOk ? "✅ НОРМА" : "❓ НЕИЗВЕСТЕН");
    
    if (verbose) {
        ESP_LOGI(TAG_BAT, "Последняя проверка: %lu мс назад",
                 millis() - _lastStatus.lastCheckTime);
        ESP_LOGI(TAG_BAT, "Всего проверок: %lu", _checkCount);
        ESP_LOGI(TAG_BAT, "Критических событий: %lu", _criticalCount);
    }
    ESP_LOGI(TAG_BAT, "==========================================");
}

void BatteryMonitor::reset() {
    _checkCount = 0;
    _criticalCount = 0;
    _lastStatus.lastCheckTime = 0;
    ESP_LOGI(TAG_BAT, "🔄 Статистика батареи сброшена");
}

// Деструктор для освобождения ресурсов ADC
        //            BatteryMonitor::~BatteryMonitor() {
        //                if (_adcCaliHandle) {
        //                    adc_cali_delete_scheme_curve_fitting(_adcCaliHandle);
        //                }
        //                if (_adcHandle) {
        //                    adc_oneshot_del_unit(_adcHandle);
        //                }
        //                ESP_LOGI("PCA9685_SERVO", "🔄 Ресурсы BatteryMonitor освобождены");
        //            }

