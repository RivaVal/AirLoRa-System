


// ### 4. `Unified_LEDC_Controller.cpp` 
//
/**
 * @file Unified_LEDC_Controller.cpp
 * @brief Реализация контроллера моторов через LEDC ESP32
 * @version 1.1.0 (исправленная)
 * @date 2026-02-13
 */
#include "Unified_LEDC_Controller.h"
#include "Config.h"
#include <driver/ledc.h>
#include "CommonTypes.h"  // 🔑 КРИТИЧЕСКИ ВАЖНО: для доступа к ControllerDebug_t
#include <esp_log.h>      // 🔑 КРИТИЧЕСКИ ВАЖНО: для работы с ESP_LOG*

//static const char* TAG_MOTOR = "MOTOR_LEDC";
extern ControllerDebug_t CDebug;   // Глобальная переменная отладки (объявлена в CommonTypes.h)

// ============================================================================
// КОНСТРУКТОР
// ============================================================================
UnifiedLEDCController::UnifiedLEDCController() : _initialized(false) {
    _lastError = "No error";
    ESP_LOGI("MOTOR_LEDC", "Конструктор UnifiedLEDCController создан");
}

// ============================================================================ CDebug.ledc
// ИНИЦИАЛИЗАЦИЯ LEDC
// ============================================================================
bool UnifiedLEDCController::begin() {
    if (_initialized) {
        ESP_LOGW("MOTOR_LEDC", "⚠️ LEDC уже инициализирован, повторная инициализация пропущена");
        return true;
    }
    
    ESP_LOGI("MOTOR_LEDC", "⚙️  Инициализация LEDC для 2 моторов...");
    ESP_LOGI("MOTOR_LEDC", "   Пины: Мотор #0 → GPIO %d, Мотор #1 → GPIO %d",
             Config::Pins::motorPins[0], Config::Pins::motorPins[1]);
    ESP_LOGI("MOTOR_LEDC", "   Частота: 1000 Гц, Разрешение: 10 бит (1024 уровня)");

    // ШАГ 1: Настройка ТАЙМЕРА (обязательно ДО каналов!)
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.timer_num = LEDC_TIMER_0;
    timer_conf.duty_resolution = LEDC_TIMER_10_BIT;  // 1024 уровня
    timer_conf.freq_hz = 1000;                       // 1 kHz для моторов
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        _lastError = "LEDC timer config failed";
        ESP_LOGE("MOTOR_LEDC", "❌ %s: код ошибки %d  (%s)", _lastError, err, 
            esp_err_to_name(err));
        return false;
    }
    ESP_LOGI("MOTOR_LEDC", "✅ Таймер LEDC настроен");

    // ШАГ 2: Настройка КАНАЛОВ (после таймера!)
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t channel_conf = {};
        channel_conf.gpio_num = Config::Pins::motorPins[i];
        channel_conf.speed_mode = LEDC_LOW_SPEED_MODE;
        channel_conf.channel = _motorChannels[i];
        channel_conf.timer_sel = LEDC_TIMER_0;
        channel_conf.duty = 0;  // Начальная мощность 0%
        channel_conf.hpoint = 0;
        
        err = ledc_channel_config(&channel_conf);
        if (err != ESP_OK) {
            _lastError = "LEDC channel config failed";
            ESP_LOGE("MOTOR_LEDC", "❌ %s для канала %d: код ошибки %d", _lastError, i, err);
            return false;
        }
        
        // Установка начального значения (обязательно!)
        err = ledc_set_duty(LEDC_LOW_SPEED_MODE, _motorChannels[i], 0);
        if (err != ESP_OK) {
            _lastError = "LEDC set duty failed";
            // ESP_LOGE("MOTOR_LEDC", "❌ %s для канала %d: код ошибки %d", _lastError, i, err);
            ESP_LOGE("MOTOR_LEDC", "❌ %s для канала %d: код ошибки %d (%s)", 
                     _lastError, i, err, esp_err_to_name(err));
            return false;
        }
        
        err = ledc_update_duty(LEDC_LOW_SPEED_MODE, _motorChannels[i]);
        if (err != ESP_OK) {
            _lastError = "LEDC update duty failed";
            // ESP_LOGE("MOTOR_LEDC", "❌ %s для канала %d: код ошибки %d", _lastError, i, err);
            ESP_LOGE("MOTOR_LEDC", "❌ %s для канала %d: код ошибки %d (%s)", 
                     _lastError, i, err, esp_err_to_name(err));
            return false;
        }
        
        ESP_LOGI("MOTOR_LEDC", "✅ Канал %d настроен (GPIO %d)", i, Config::Pins::motorPins[i]);
    }

    _initialized = true;
    ESP_LOGI("MOTOR_LEDC", "✅ LEDC успешно инициализирован для 2 моторов");
    return true;
}

// ============================================================================
// УПРАВЛЕНИЕ МОТОРАМИ
// ============================================================================
bool UnifiedLEDCController::setMotorPower(uint8_t motorIndex, float powerPercent) {
    if (!_initialized) {
        ESP_LOGE("MOTOR_LEDC", "❌ setMotorPower: контроллер не инициализирован");
        return false;
    }
    
    if (motorIndex >= 2) {
        ESP_LOGE("MOTOR_LEDC", "❌ setMotorPower: неверный индекс мотора (%u), допустимо 0-1", motorIndex);
        return false;
    }

    // Ограничение мощности для безопасности
    if (powerPercent < 0.0f) powerPercent = 0.0f;
    if (powerPercent > 100.0f) powerPercent = 100.0f;

    // Преобразование % в значение 10-битного разрешения (0-1023)
    uint32_t duty = static_cast<uint32_t>((powerPercent * 1023.0f) / 100.0f);
    
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, _motorChannels[motorIndex], duty);
    if (err != ESP_OK) {
        // ESP_LOGE("MOTOR_LEDC", "❌ ledc_set_duty для мотора %u не удался: %d", motorIndex, err);
        ESP_LOGE("MOTOR_LEDC", "❌ ledc_set_duty для мотора %u не удался: %d (%s)", 
                 motorIndex, err, esp_err_to_name(err));
        return false;
    }
    
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, _motorChannels[motorIndex]);
    if (err != ESP_OK) {
        // ESP_LOGE("MOTOR_LEDC", "❌ ledc_update_duty для мотора %u не удался: %d", motorIndex, err);
        ESP_LOGE("MOTOR_LEDC", "❌ ledc_update_duty для мотора %u не удался: %d (%s)", 
                 motorIndex, err, esp_err_to_name(err));
        return false;
    }

    if (CDebug.motors && CDebug.ledc ) {
        ESP_LOGD("MOTOR_LEDC", "Мотор #%u: мощность=%.1f%% → duty=%u/1023", 
                 motorIndex, powerPercent, duty);
    }
    
    return true;
}

void UnifiedLEDCController::stopAllMotors() {
    if (!_initialized) {
        ESP_LOGW("MOTOR_LEDC", "⚠️ stopAllMotors: контроллер не инициализирован, операция пропущена");
        return;
    }
    
    setMotorPower(0, 0.0f);
    setMotorPower(1, 0.0f);
    ESP_LOGI("MOTOR_LEDC", "🛑 Все моторы остановлены (0%% мощности)");
}

// ============================================================================
// БЕЗОПАСНЫЙ ТЕСТ МОТОРОВ
// ============================================================================
bool UnifiedLEDCController::runMotorTest(uint32_t durationMs) {
    if (!_initialized) {
        ESP_LOGE("MOTOR_LEDC", "❌ Тест моторов: контроллер не инициализирован!");
        return false;
    }

    ESP_LOGI("MOTOR_LEDC", "▶️  ЗАПУСК БЕЗОПАСНОГО ТЕСТА МОТОРОВ");
    ESP_LOGW("MOTOR_LEDC", "⚠️  Максимальная мощность ограничена 35%% для безопасности!");
    
    // Мотор 0: плавное увеличение до 35%
    ESP_LOGI("MOTOR_LEDC", "  Мотор #0: 0%% → 35%%");
    for (int p = 0; p <= 35; p += 5) {
        //setMotorPower(0, static_cast<float>(p));
        if (!setMotorPower(0, static_cast<float>(p))) {
            ESP_LOGE("MOTOR_LEDC", "❌ Ошибка установки мощности мотора #0 на %d%%", p);
            stopAllMotors();
            return false;
        }
        delay(80);
    }
    delay(600);
    
    // Мотор 0: плавное снижение до 0%
    ESP_LOGI("MOTOR_LEDC", "  Мотор #0: 35%% → 0%%");
    for (int p = 35; p >= 0; p -= 5) {
        //setMotorPower(0, static_cast<float>(p));
        if (!setMotorPower(0, static_cast<float>(p))) {
            ESP_LOGE("MOTOR_LEDC", "❌ Ошибка установки мощности мотора #0 на %d%%", p);
            stopAllMotors();
            return false;
        }
        delay(80);
    }
    delay(300);

    // Мотор 1: плавное увеличение до 35%
    ESP_LOGI("MOTOR_LEDC", "  Мотор #1: 0%% → 35%%");
    for (int p = 0; p <= 35; p += 5) {
        //setMotorPower(1, static_cast<float>(p));
        if (!setMotorPower(1, static_cast<float>(p))) {
            ESP_LOGE("MOTOR_LEDC", "❌ Ошибка установки мощности мотора #1 на %d%%", p);
            stopAllMotors();
            return false;
        }
        delay(80);
    }
    delay(600);
    
    // Мотор 1: плавное снижение до 0%
    ESP_LOGI("MOTOR_LEDC", "  Мотор #1: 35%% → 0%%");
    for (int p = 35; p >= 0; p -= 5) {
        setMotorPower(1, static_cast<float>(p));
        setMotorPower(1, static_cast<float>(p));
        delay(80);
    }
    delay(300);

    // Совместная работа на 20%
    ESP_LOGI("MOTOR_LEDC", "  Оба мотора: 20%% мощности");
    setMotorPower(0, 20.0f);
    setMotorPower(1, 20.0f);
    delay(800);

    // КРИТИЧЕСКИ ВАЖНО: Возврат в нулевую мощность!
    stopAllMotors();
    
    ESP_LOGI("MOTOR_LEDC", "✅ Тест моторов завершен. Мощность = 0%%");
    return true;
}
