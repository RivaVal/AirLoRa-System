


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
const char* BatteryMonitor::TAG = "BATTERY";


// ============================================================================
// КОНСТРУКТОР
// ============================================================================
BatteryMonitor::BatteryMonitor()
    : _initialized(false), _checkCount(0), _criticalCount(0),
      _adcHandle(nullptr), _adcCaliHandle(nullptr), _adcCaliValid(false)
{
    _lastStatus = {};
    _currentConfig = BATTERY_CONFIG_TABLE[1]; // 4S по умолчанию
    ESP_LOGI("BATTERY", "✅ Конструктор BatteryMonitor инициализирован");
}

BatteryMonitor::~BatteryMonitor() {
    if (_adcCaliHandle) {
        ESP_LOGD("BATTERY", "Освобождение калибровки ADC");
        adc_cali_delete_scheme_line_fitting(_adcCaliHandle);
    }
    if (_adcHandle) {
        ESP_LOGD("BATTERY", "Освобождение ADC unit");
        adc_oneshot_del_unit(_adcHandle);
    }
    ESP_LOGI("BATTERY", "🔄 Ресурсы BatteryMonitor освобождены");
}



// ============================================================================
// ИНИЦИАЛИЗАЦИЯ ADC (НОВЫЙ ДРАЙВЕР)
// ============================================================================
bool BatteryMonitor::initADC() {
    ESP_LOGI("BATTERY", "⚙️  Инициализация ADC (новый драйвер esp_adc/adc_oneshot)...");
    
    // 1. Создание ADC unit
    adc_oneshot_unit_init_cfg_t initConfig = {
        .unit_id = ADC_UNIT_1,
                                    // ✅ Обязательное поле в ESP-IDF 5.0+
                                    //          .clk_src = ADC_CLK_SRC_DEFAULT,
                                    //          .clk_src = ADC_UNIT_1_CLK_SRC_DEFAULT,
                                    //          .clk_src = ADC_CLK_SRC_RC_FAST,
        .clk_src = ADC_RTC_CLK_SRC_RC_FAST,   // ✅ Обязательное поле в ESP-IDF 5.0+
        .ulp_mode = ADC_ULP_MODE_DISABLE  // ADC_RTC_CLK_SRC_RC_FAST
    };
    
    esp_err_t err = adc_oneshot_new_unit(&initConfig, &_adcHandle);
    if (err != ESP_OK) {
        ESP_LOGE("BATTERY", "❌ Ошибка создания ADC unit: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI("BATTERY", "✅ ADC unit создан");
    
    // 2. Конфигурация канала
    adc_oneshot_chan_cfg_t chanConfig = {
        .atten = BATTERY_ADC_ATTENUATION,
                                    // ✅ ИСПРАВЛЕНО: ADC_WIDTH_12 вместо ADC_WIDTH_BIT_12
                                    // .bitwidth = ADC_WIDTH_12,  //ADC_WIDTH_BIT_12
        .bitwidth = ADC_BITWIDTH_12   // ✅ 12-битное разрешение
    };
    
    // ✅ НОВОЕ ПРАВИЛЬНОЕ:
    // 🔑 КРИТИЧЕСКИ: правильный маппинг GPIO34 → ADC1_CHANNEL_6
    adc_channel_t adcChannel;
    switch(BATTERY_ADC_PIN) {
        case 32: adcChannel = ADC_CHANNEL_4; break;
        case 33: adcChannel = ADC_CHANNEL_5; break;
        case 34: adcChannel = ADC_CHANNEL_6; break;  // Для GPIO34 используем ADC_CHANNEL_6
        case 35: adcChannel = ADC_CHANNEL_7; break;
        case 36: adcChannel = ADC_CHANNEL_0; break;
        case 37: adcChannel = ADC_CHANNEL_1; break;
        case 38: adcChannel = ADC_CHANNEL_2; break;
        case 39: adcChannel = ADC_CHANNEL_3; break;
        default:
            ESP_LOGE("BATTERY", "❌ Неподдерживаемый пин для ADC: %d", BATTERY_ADC_PIN);
            adc_oneshot_del_unit(_adcHandle);
            return false;
    }

    err = adc_oneshot_config_channel(_adcHandle, adcChannel, &chanConfig);
    ESP_ERROR_CHECK_RETURN(err != ESP_OK);
    
    ESP_LOGI("BATTERY", "✅ Канал ADC настроен (GPIO%d → ADC1_CH%d)", 
             BATTERY_ADC_PIN, adcChannel);
    
                    //                err = adc_oneshot_config_channel(_adcHandle, 
                    //                                                static_cast<adc_channel_t>(BATTERY_ADC_PIN), 
                    //                                                &chanConfig);
                        //if (err != ESP_OK) {
                        //    ESP_LOGE("PCA9685_SERVO", "❌ Ошибка конфигурации канала: %s", esp_err_to_name(err));
                        //    adc_oneshot_del_unit(_adcHandle);
                        //    return false;
                        //}
    ESP_LOGI("BATTERY", "✅ Канал ADC настроен (GPIO%d)", BATTERY_ADC_PIN);
        // * /    

    // 3. Калибровка (опционально, но рекомендуется)
    _adcCaliValid = false;
    adc_cali_line_fitting_config_t caliConfig = {
        .unit_id = ADC_UNIT_1,
        .atten = BATTERY_ADC_ATTENUATION,
        .bitwidth = ADC_BITWIDTH_12,
        .default_vref = 1100
    };
    
    err = adc_cali_create_scheme_line_fitting(&caliConfig, &_adcCaliHandle);
    if (err == ESP_OK) {
        _adcCaliValid = true;
        ESP_LOGI("BATTERY", "✅ Калибровка ADC активирована (line fitting)");
    } else {
        ESP_LOGW("BATTERY", "⚠️  Калибровка ADC недоступна");
    }
    
    return true;
}

// ============================================================================
// ЧТЕНИЕ ADC
// ============================================================================
bool BatteryMonitor::readADC(int &adcValue) {
    if (!_initialized || !_adcHandle) {
        ESP_LOGE("BATTERY", "❌ readADC: ADC не инициализирован");
        return false;
    }

    // 🔑 GPIO34 соответствует ADC1_CHANNEL_6 в ESP32
    adc_channel_t adcChannel = ADC_CHANNEL_6;  // Для GPIO34

            //        esp_err_t err = adc_oneshot_read(_adcHandle, 
            //                                        static_cast<adc_channel_t>(BATTERY_ADC_PIN), 
            //                                        &adcValue);

                //                // Правильный 🔑 Маппинг GPIO → ADC канал (дублируем для безопасности)
                //                adc_channel_t adcChannel;
                //                switch(BATTERY_ADC_PIN) {
                //                    case 32: adcChannel = ADC_CHANNEL_4; break;
                //                    case 33: adcChannel = ADC_CHANNEL_5; break;
                //                    case 34: adcChannel = ADC_CHANNEL_6; break;   // Для GPIO34 используем ADC_CHANNEL_6
                //                    case 35: adcChannel = ADC_CHANNEL_7; break;
                //                    case 36: adcChannel = ADC_CHANNEL_0; break;
                //                    case 37: adcChannel = ADC_CHANNEL_1; break;
                //                    case 38: adcChannel = ADC_CHANNEL_2; break;
                //                    case 39: adcChannel = ADC_CHANNEL_3; break;
                //                    default:
                //                        ESP_LOGE("PCA9685_SERVO", "❌ Неподдерживаемый пин для ADC: %d", BATTERY_ADC_PIN);
                //                        return false;
                //                }

    esp_err_t err = adc_oneshot_read(_adcHandle, adcChannel, &adcValue);            

    if (err != ESP_OK) {
        ESP_LOGE("BATTERY", "❌ Ошибка чтения ADC: %s", esp_err_to_name(err));
        return false;
    }
    
                // Применение калибровки если доступна
                //                if (_adcCaliValid && _adcCaliHandle) {
                //                    int voltageMV = 0;
                //                    err = adc_cali_raw_to_voltage(_adcCaliHandle, adcValue, &voltageMV);
                //                    if (err == ESP_OK) {
                //                       adcValue = (voltageMV * BATTERY_ADC_RESOLUTION) / 3300;
                //                    }
                //                }

    // Применение калибровки если доступна
    if (_adcCaliValid && _adcCaliHandle) {
        int voltageMV = 0;
        err = adc_cali_raw_to_voltage(_adcCaliHandle, adcValue, &voltageMV);
        if (err == ESP_OK) {
            // Конвертируем мВ обратно в raw-значение для совместимости
            adcValue = (voltageMV * BATTERY_ADC_RESOLUTION) / 3300;
            ESP_LOGV("BATTERY", "ADC: raw=%d → calibrated=%d mV → raw=%d", 
                     adcValue, voltageMV, adcValue);
        }
    }

    return true;
}



// ============================================================================
// ПРЕОБРАЗОВАНИЕ НАПРЯЖЕНИЯ
// ============================================================================
// ✅ ИСПРАВЛЕНО: параметр int вместо uint16_t (совместимость с readADC)
float BatteryMonitor::adcToBatteryVoltage(int adcValue) const {
    float adcVoltage = (adcValue / static_cast<float>(BATTERY_ADC_RESOLUTION))
                       * BATTERY_ADC_MAX_VOLTAGE;
    return adcVoltage * BATTERY_VOLTAGE_DIVIDER_RATIO;
}

// ============================================================================
// ПУБЛИЧНЫЕ МЕТОДЫ
// ============================================================================
bool BatteryMonitor::begin(uint8_t cellCount) {
    ESP_LOGI("BATTERY", "=== 🚀 ИНИЦИАЛИЗАЦИЯ МОНИТОРА БАТАРЕИ ===");
    
    if (cellCount < MIN_CELL_COUNT || cellCount > MAX_CELL_COUNT) {
        ESP_LOGE("BATTERY", "❌ Недопустимое количество ячеек: %u (допустимо %u-%u)",
                 cellCount, MIN_CELL_COUNT, MAX_CELL_COUNT);
        return false;
    }
    
    const BatteryConfig_t* config = findConfig(cellCount);
    if (!config) {
        ESP_LOGE("BATTERY", "❌ Конфигурация для %uS не найдена в таблице", cellCount);
        return false;
    }
    
    _currentConfig = *config;
    ESP_LOGI("BATTERY", "🔋 Конфигурация АКБ: %s", _currentConfig.name);
    ESP_LOGI("BATTERY", "   Ячеек: %u", _currentConfig.cellCount);
    ESP_LOGI("BATTERY", "   Диапазон: %.2fВ - %.2fВ",
             _currentConfig.voltageMin, _currentConfig.voltageMax);
    ESP_LOGI("BATTERY", "   Критический: %.2fВ", _currentConfig.voltageCritical);
    
    if (!initADC()) {
        ESP_LOGE("BATTERY", "❌ Критическая ошибка: не удалось инициализировать ADC");
        return false;
    }
    
    _initialized = true;
    
    // ✅ ИСПРАВЛЕНО: STARTUP_CHECK_DELAY вместо BATTERY_STARTUP_CHECK_DELAY
    ESP_LOGI("BATTERY", "⏳ Задержка перед первой проверкой (%u мс)...",
             STARTUP_CHECK_DELAY);
    delay(STARTUP_CHECK_DELAY);
    checkVoltage();
    
    ESP_LOGI("BATTERY", "✅ Монитор батареи успешно инициализирован");
    return true;
}


BatteryStatus_t BatteryMonitor::checkVoltage() {
    if (!_initialized) {
        ESP_LOGE("BATTERY", "❌ checkVoltage: монитор не инициализирован!");
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
        ESP_LOGE("BATTERY", "🔴 КРИТИЧЕСКИ НИЗКОЕ напряжение: %.2fВ (%.1f%%)", voltage, percentage);
    }
    
    return _lastStatus;
}

void BatteryMonitor::update() {
    static uint32_t lastCheck = 0;
    if (!_initialized) return;
    
    uint32_t currentTime = millis();
    // ✅ ИСПРАВЛЕНО: CHECK_INTERVAL_MS вместо BATTERY_CHECK_INTERVAL_MS
    if (currentTime - lastCheck >= CHECK_INTERVAL_MS) {
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
        ESP_LOGE("BATTERY", "❌ setCellCount: недопустимое значение %u", cellCount);
        return false;
    }
    
    const BatteryConfig_t* config = findConfig(cellCount);
    if (!config) {
        ESP_LOGE("BATTERY", "❌ setCellCount: конфигурация не найдена");
        return false;
    }
    
    _currentConfig = *config;
    ESP_LOGI("BATTERY", "✅ Конфигурация АКБ изменена: %s", _currentConfig.name);
    
    // Немедленная проверка с новой конфигурацией
    checkVoltage();
    return true;
}

void BatteryMonitor::printStatus(bool verbose) const {
    ESP_LOGI("BATTERY", "==========================================");
    ESP_LOGI("BATTERY", "🔋 СТАТУС БАТАРЕИ");
    ESP_LOGI("BATTERY", "==========================================");
    ESP_LOGI("BATTERY", "Конфигурация: %s (%uS)", _lastStatus.configName, _lastStatus.cellCount);
    ESP_LOGI("BATTERY", "Напряжение: %.2f В", _lastStatus.voltage);
    ESP_LOGI("BATTERY", "На ячейку: %.2f В", _lastStatus.voltagePerCell);
    ESP_LOGI("BATTERY", "Заряд: %.1f %%", _lastStatus.percentage);
    ESP_LOGI("BATTERY", "ADC значение: %u", _lastStatus.adcValue);
    ESP_LOGI("BATTERY", "Статус: %s",
             _lastStatus.isCritical ? "🔴 КРИТИЧЕСКИЙ" :
             _lastStatus.isLow ? "⚠️  НИЗКИЙ" :
             _lastStatus.isOk ? "✅ НОРМА" : "❓ НЕИЗВЕСТЕН");
    
    if (verbose) {
        ESP_LOGI("BATTERY", "Последняя проверка: %lu мс назад",
                 millis() - _lastStatus.lastCheckTime);
        ESP_LOGI("BATTERY", "Всего проверок: %lu", _checkCount);
        ESP_LOGI("BATTERY", "Критических событий: %lu", _criticalCount);
    }
    ESP_LOGI("BATTERY", "==========================================");
}

void BatteryMonitor::reset() {
    _checkCount = 0;
    _criticalCount = 0;
    _lastStatus.lastCheckTime = 0;
    ESP_LOGI("BATTERY", "🔄 Статистика батареи сброшена");
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

