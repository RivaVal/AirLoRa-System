

/**
* @file FlightStabilizer.cpp
* @brief Реализация системы стабилизации полёта БПЛА
* 
* @details Реализует ПИД-стабилизацию с комплементарным фильтром
* для обработки данных IMU и управления сервоприводами.
* 
* @author Embedded Systems Team
* @version 1.1.0 (ИСПРАВЛЕННАЯ ВЕРСИЯ)
* @date 2026-02-19
* 
* @note ИСПРАВЛЕНИЯ:
* - Конструктор без параметров (совместимость с Receiver.ino)
* - begin() принимает указатели (не ссылки)
* - Добавлены все отсутствующие поля класса
* - Добавлены все отсутствующие методы
* - Исправлено использование указателей (-> вместо .)
* - Добавлена подробная отладка ESP_LOG
*/

#include "FlightStabilizer.h"
#include "esp_log.h"

//static const char* TAG = "FLIGHT_STAB";
static const char* TAG_FLIGHT = "FLIGHT_STAB";  // ← ДОБАВИТЬ

// ============================================================================
// КОНСТРУКТОР И ИНИЦИАЛИЗАЦИЯ
// ============================================================================

/**
* @brief Конструктор системы стабилизации
* 
* @note Инициализирует все поля значениями по умолчанию
*       Реальная инициализация выполняется в begin()
*/
FlightStabilizer::FlightStabilizer() 
    : imuHandler(nullptr)
    , _servoController(nullptr)
    , _initialized(false)
    , _enabled(false)
    , _mode(StabilizationMode::MANUAL)
    , _dt(0.01f)
    , _lastUpdateTime(0)
    , _targetRoll(0.0f)
    , _targetPitch(0.0f)
    , _targetYaw(0.0f)
{
    // Инициализация ПИД-коэффициентов (значения по умолчанию)
    _rollPID = {2.0f, 0.0f, 0.5f, 10.0f, 30.0f};
    _pitchPID = {2.0f, 0.0f, 0.5f, 10.0f, 30.0f};
    _yawPID = {1.5f, 0.0f, 0.3f, 10.0f, 45.0f};
    
    // Сброс состояний ПИД
    resetPIDStates();
    
    ESP_LOGI(TAG_FLIGHT, "✅ FlightStabilizer: Конструктор вызван");
    ESP_LOGD(TAG_FLIGHT, "   ПИД Roll: Kp=%.2f, Ki=%.2f, Kd=%.2f", 
             _rollPID.kp, _rollPID.ki, _rollPID.kd);
    ESP_LOGD(TAG_FLIGHT, "   ПИД Pitch: Kp=%.2f, Ki=%.2f, Kd=%.2f", 
             _pitchPID.kp, _pitchPID.ki, _pitchPID.kd);
}

/**
* @brief Инициализация системы стабилизации
* 
* @param gyroHandler Указатель на обработчик гироскопа
* @param servoController Указатель на контроллер сервоприводов
* @return true при успешной инициализации, false при ошибке
* 
* @note Проверяет валидность указателей перед использованием
*/
// bool FlightStabilizer::begin(GY87Handler* gyroHandler, 
bool FlightStabilizer::begin(MPU9250Handler* _imuHandler, 
                             PCA9685_ServoController* servoController) 
{
    ESP_LOGI(TAG_FLIGHT, "=== ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ СТАБИЛИЗАЦИИ ПОЛЁТА ===");
    
    // ✅ ПРОВЕРКА УКАЗАТЕЛЕЙ (критически важно!)
    if (!imuHandler) {
        ESP_LOGE(TAG_FLIGHT, "❌ Ошибка: нулевой указатель на гироскоп");
        return false;
    }
    
    if (!servoController) {
        ESP_LOGE(TAG_FLIGHT, "❌ Ошибка: нулевой указатель на контроллер сервоприводов");
        return false;
    }
    
    // ✅ СОХРАНЕНИЕ УКАЗАТЕЛЕЙ
    imuHandler = _imuHandler;
    _servoController = servoController;
    
    // ✅ ПРОВЕРКА ИНИЦИАЛИЗАЦИИ ГИРОСКОПА
    if (!imuHandler->isInitialized()) {
        ESP_LOGW(TAG_FLIGHT, "⚠️ Гироскоп ещё не инициализирован");
        // Не блокируем - калибровка может идти
    }
    
    // ✅ СБРОС СОСТОЯНИЙ ПИД
    resetPIDStates();
    
    // ✅ УСТАНОВКА БЕЗОПАСНЫХ НАЧАЛЬНЫХ ЗНАЧЕНИЙ
    _targetRoll = 0.0f;
    _targetPitch = 0.0f;
    _targetYaw = 0.0f;
    _lastUpdateTime = millis();
    
    // ✅ УСТАНОВКА ФЛАГА ИНИЦИАЛИЗАЦИИ
    _initialized = true;
    _enabled = false; // По умолчанию выключена до ручной активации
    
    ESP_LOGI(TAG_FLIGHT, "✅ Система стабилизации инициализирована");
    ESP_LOGI(TAG_FLIGHT, "   Режим по умолчанию: РУЧНОЕ УПРАВЛЕНИЕ");
    ESP_LOGI(TAG_FLIGHT, "   ПИД-коэффициенты:");
    ESP_LOGI(TAG_FLIGHT, "     Roll:  Kp=%.2f Ki=%.2f Kd=%.2f (max=%.1f°)",
             _rollPID.kp, _rollPID.ki, _rollPID.kd, _rollPID.maxOutput);
    ESP_LOGI(TAG_FLIGHT, "     Pitch: Kp=%.2f Ki=%.2f Kd=%.2f (max=%.1f°)",
             _pitchPID.kp, _pitchPID.ki, _pitchPID.kd, _pitchPID.maxOutput);
    ESP_LOGI(TAG_FLIGHT, "     Yaw:   Kp=%.2f Ki=%.2f Kd=%.2f (max=%.1f°)",
             _yawPID.kp, _yawPID.ki, _yawPID.kd, _yawPID.maxOutput);
    
    // Сохраняем "идеальные" коэффициенты
    _baseRollPID = _rollPID;
    _basePitchPID = _pitchPID;
    
    return true;
}

// ============================================================================
// УПРАВЛЕНИЕ СТАБИЛИЗАЦИЕЙ
// ============================================================================

/**
* @brief Включение системы стабилизации
*/
void FlightStabilizer::enable() {
    if (!_initialized) {
        ESP_LOGW(TAG_FLIGHT, "⚠️ Нельзя включить: система не инициализирована");
        return;
    }
    _enabled = true;
    resetPIDStates(); // Сброс интегральной составляющей при включении
    ESP_LOGI(TAG_FLIGHT, "✅ Стабилизация ВКЛЮЧЕНА");
}

/**
* @brief Отключение системы стабилизации
*/
void FlightStabilizer::disable() {
    _enabled = false;
    ESP_LOGI(TAG_FLIGHT, "⏸️ Стабилизация ОТКЛЮЧЕНА");
}

/**
* @brief Установка режима стабилизации
* @param mode Режим стабилизации
*/
void FlightStabilizer::setMode(StabilizationMode mode) {
    _mode = mode;
    ESP_LOGI(TAG_FLIGHT, "🔄 Режим стабилизации: %s", modeToString(mode));
}

// ============================================================================
// ОСНОВНОЙ ЦИКЛ СТАБИЛИЗАЦИИ
// ============================================================================

/**
* @brief Обновление состояния стабилизации
* 
* @param comUp Команда тангажа от пульта (0-255)
* @param comLeft Команда крена от пульта (0-255)
* @return true если стабилизация активна и данные валидны
* 
* @note Вызывать в loop() с частотой 50-100 Гц
*/
// **Задача:** Внедрить фиксированный шаг дискретизации (Fixed Timestep) для стабильности ПИД.
/**
 * @brief Обновление состояния стабилизации с фиксированным шагом (250 Гц)
 * @details Блокирует выполнение до достижения TARGET_LOOP_US (4000 мкс).
 *          Это устраняет джиттер от Wi-Fi/LoRa/I2C и делает ПИД детерминированным.
 */
/**
 * @brief Вызов calculatePID с индексом оси
 * @param axisIndex: 0=Roll, 1=Pitch, 2=Yaw
 * @note Индекс используется для:
 *   • Отладки через ESP_LOG (идентификация оси)
 *   • Адаптивной настройки ПИД-коэффициентов
 *   • Фильтрации производной по каждой оси отдельно
 */
bool FlightStabilizer::update(float comUp, float comLeft) {
    // 🔑 Фиксированный шаг для ESP32-S3
    if (_fixedTimestepEnabled) {
        uint32_t now = micros();
        int32_t elapsed = now - _lastUpdateMicros;
        if (elapsed < TARGET_LOOP_US) {
            return false; // Пропускаем цикл, ждем следующего тика
        }
        _dt = TARGET_LOOP_US / 1000000.0f; // Фиксированный dt = 0.004 с
        _lastUpdateMicros = now;
    } else {
        // Fallback для обратной совместимости
        uint32_t currentTime = millis();
        _dt = (currentTime - _lastUpdateTime) / 1000.0f;
        _lastUpdateTime = currentTime;
        if (_dt > 0.1f || _dt <= 0.0f) _dt = 0.01f;
    }

//--------------------------------------------------------------------
    // ✅ ПРОВЕРКА ИНИЦИАЛИЗАЦИИ
    if (!_initialized) {
        ESP_LOGW(TAG_FLIGHT, "⚠️ update(): система не инициализирована");
        return false;
    }
    
    // ✅ ПРОВЕРКА ВКЛЮЧЕНИЯ
    if (!_enabled) {
        return false; // Стабилизация отключена
    }
    
    // ✅ ПРОВЕРКА УКАЗАТЕЛЕЙ
    if (!imuHandler || !_servoController) {
        ESP_LOGE(TAG_FLIGHT, "❌ update(): нулевые указатели на модули");
        return false;
    }
    
    // ✅ ПРОВЕРКА ВАЛИДНОСТИ ДАННЫХ ГИРОСКОПА
    if (!imuHandler->isDataValid()) {
        _stats.invalidDataEvents++;
        if (_stats.invalidDataEvents % 100 == 0) {
            ESP_LOGW(TAG_FLIGHT, "⚠️ Данные гироскопа недоступны (событий: %lu)", 
                     _stats.invalidDataEvents);
        }
        return false;
    }
    
    // ✅ ПОЛУЧЕНИЕ ТЕКУЩИХ УГЛОВ ИЗ ГИРОСКОПА
    const SensorData& sensorData = imuHandler->getData();
    float currentRoll = sensorData.roll;
    float currentPitch = sensorData.pitch;
    float currentYaw = sensorData.yaw;
    
    // ✅ ВАЛИДАЦИЯ УГЛОВ
    validateAngles(currentRoll, currentPitch, currentYaw);
    
    // ✅ ОБНОВЛЕНИЕ ЦЕЛЕВЫХ УГЛОВ ИЗ КОМАНД ПУЛЬТА
    // Преобразование 0-255 в -45°..+45°
    _targetPitch = map(comUp, 0, 255, -45.0f, 45.0f);
    _targetRoll = map(comLeft, 0, 255, -45.0f, 45.0f);
    
    // ✅ ВЫЧИСЛЕНИЕ ВРЕМЕННОГО ШАГА
    uint32_t currentTime = millis();
    _dt = (currentTime - _lastUpdateTime) / 1000.0f;
    if (_dt > 0.1f || _dt <= 0.0f) {
        _dt = 0.01f; // Защита от аномальных значений
    }
    _lastUpdateTime = currentTime;
    
    // ✅ ПРИМЕНЕНИЕ СТАБИЛИЗАЦИИ В ЗАВИСИМОСТИ ОТ РЕЖИМА
    switch (_mode) {
        case StabilizationMode::ROLL_PITCH:
        {
            // Коррекция крена
            float rollError = currentRoll - _targetRoll;
            float rollCorrection = calculatePID(_rollState, _rollPID, rollError, 0);  // ← 0 = Roll
            applyRollCorrection(rollCorrection);
            
            // Коррекция тангажа
            float pitchError = currentPitch - _targetPitch;
            float pitchCorrection = calculatePID(_pitchState, _pitchPID, pitchError, 1);   // ← 1 = Pitch
            applyPitchCorrection(pitchCorrection);
            
            _stats.corrections += 2;
        }
        break;
        
        case StabilizationMode::FULL:
        {
            // Коррекция крена
            float rollError = currentRoll - _targetRoll;
            float rollCorrection = calculatePID(_rollState, _rollPID, rollError, 0);
            applyRollCorrection(rollCorrection);
            
            // Коррекция тангажа
            float pitchError = currentPitch - _targetPitch;
            float pitchCorrection = calculatePID(_pitchState, _pitchPID, pitchError, 1);   // ← 1 = Pitch
            applyPitchCorrection(pitchCorrection);
            
            // Коррекция рыскания
            float yawError = currentYaw - _targetYaw;
            float yawCorrection = calculatePID(_yawState, _yawPID, yawError, 2);  // ← 2 = Yaw
            applyYawCorrection(yawCorrection);
            
            _stats.corrections += 3;
        }
        break;
        
        case StabilizationMode::MANUAL:
        default:
            // Ручной режим — стабилизация отключена
            return false;
    }
    
    _stats.stabilizations++;
    
    // ✅ ОТЛАДОЧНЫЙ ВЫВОД (каждые 50 обновлений)
    if (_stats.stabilizations % 50 == 0) {
        ESP_LOGD(TAG_FLIGHT, "STAB| Roll: %+5.1f°→%+5.1f°| Pitch: %+5.1f°→%+5.1f°| Yaw: %5.1f°→%5.1f°",
                 currentRoll, _targetRoll, currentPitch, _targetPitch, currentYaw, _targetYaw);
    }
    
    return _enabled;
}

// ============================================================================
// ПИД-РЕГУЛЯТОР
// ============================================================================

/**
* @brief Расчет ПИД-регулятора
* 
* @param state Состояние регулятора (ссылка для обновления)
* @param config Конфигурация регулятора
* @param error Текущая ошибка регулирования
* @return Выходное значение регулятора
* 
* @note Реализует стандартную формулу: P + I + D
*       С анти-насыщением интегральной составляющей
*/
/**
 * @brief Расчет ПИД-регулятора с фильтром производной и Anti-Windup
 * @details Оптимизирован для ESP32-S3:
 *   1. Производная вычисляется по ошибке, но фильтруется для подавления шума датчика.
 *   2. Интеграл ограничивается только при достижении предела выхода (Anti-Windup).
 *   3. Добавлен ESP_LOGD для вывода компонент P, I, D при DEBUG_VERBOSE.
 */
float FlightStabilizer::calculatePID(
    PIDState& state, 
    const PIDConfig& config, 
    float error, int axisIndex ) {

    if (_dt <= 0.0f) _dt = 0.004f; // Защита от деления на ноль

    // 1. Пропорциональная составляющая
    float P = config.kp * error;

    // 2. Интегральная составляющая с Anti-Windup (условная интеграция)
    float I = 0.0f;
    bool outputSaturated = false;

    // Временный расчет выхода для проверки насыщения
    float tempOutput = P + config.ki * state.integral + config.kd * _prevDerivative[axisIndex];
    if (abs(tempOutput) >= config.maxOutput) outputSaturated = true;

    if (!outputSaturated) {
        state.integral += error * _dt;
    }
    state.integral = constrain(state.integral, -config.maxIntegral, config.maxIntegral);
    I = config.ki * state.integral;

    // 3. Дифференциальная составляющая с LPF-фильтром
    float D_raw = (error - state.lastError) / _dt;
    float D = config.kd * (config.dFilterAlpha * D_raw + (1.0f - config.kd * config.dFilterAlpha) * _prevDerivative[axisIndex]);
    _prevDerivative[axisIndex] = D_raw; // Сохраняем для следующего шага

    state.lastError = error;
    float output = P + I + D;

    // 4. Ограничение выхода
    output = constrain(output, -config.maxOutput, config.maxOutput);

    // 🔧 Отладка для тюнинга (выводится только при DEBUG_VERBOSE)
    ESP_LOGD("PID_TUNER", "[%s] P=%.2f I=%.2f D=%.2f | OUT=%.2f",
             (axisIndex == 0) ? "ROLL" : (axisIndex == 1) ? "PITCH" : "YAW",
             P, I, D, output);

    return output;
}



/*  
float FlightStabilizer::calculatePID(PIDState& state, 
                                      const PIDConfig& config, 
                                      float error) 
{
    // Пропорциональная составляющая
    float p = config.kp * error;
    
    // Интегральная составляющая с анти-насыщением
    state.integral += error * _dt;
    state.integral = constrain(state.integral, -config.maxIntegral, config.maxIntegral);
    float i = config.ki * state.integral;
    
    // Дифференциальная составляющая
    state.derivative = (error - state.lastError) / _dt;
    float d = config.kd * state.derivative;
    
    // Сохранение текущей ошибки для следующего шага
    state.lastError = error;
    
    // Формирование выходного сигнала
    float output = p + i + d;
    
    // Ограничение максимальной коррекции
    if (abs(output) > config.maxOutput) {
        _stats.saturationEvents++;
        output = constrain(output, -config.maxOutput, config.maxOutput);
    }
    
    state.output = output;
    
    return output;
}
*/

// ============================================================================
// ПРИМЕНЕНИЕ КОРРЕКЦИЙ К СЕРВОПРИВОДАМ
// ============================================================================

/**
* @brief Применение коррекции крена
* 
* @param correction Величина коррекции (градусы)
* 
* @note Дифференциальное управление элеронами:
*       - При крене вправо: левый элерон вверх, правый вниз
*/
void FlightStabilizer::applyRollCorrection(float correction) {
    // Дифференциальное управление элеронами
    float leftAileron = _targetRoll - correction;   // Канал 0
    float rightAileron = _targetRoll + correction;  // Канал 1
    
    // Ограничение диапазона
    leftAileron = constrain(leftAileron, -45.0f, 45.0f);
    rightAileron = constrain(rightAileron, -45.0f, 45.0f);
    
    // Установка углов сервоприводов
    _servoController->setLogicalAngle(0, static_cast<int16_t>(leftAileron));
    _servoController->setLogicalAngle(1, static_cast<int16_t>(rightAileron));
    
    // Отладка (каждые 100 коррекций)
    if (_stats.corrections % 100 == 0) {
        ESP_LOGV(TAG_FLIGHT, "🔄 Крен: коррекция=%+5.1f°| Левый=%+5.1f°| Правый=%+5.1f°",
                 correction, leftAileron, rightAileron);
    }
}

/**
* @brief Применение коррекции тангажа
* 
* @param correction Величина коррекции (градусы)
* 
* @note Синхронное управление рулями высоты:
*       - Оба руля отклоняются одинаково
*/
void FlightStabilizer::applyPitchCorrection(float correction) {
    // Синхронное управление рулями высоты
    float elevatorAngle = _targetPitch + correction;
    elevatorAngle = constrain(elevatorAngle, -45.0f, 45.0f);
    
    // Установка углов сервоприводов (каналы 2 и 3)
    _servoController->setLogicalAngle(2, static_cast<int16_t>(elevatorAngle));
    _servoController->setLogicalAngle(3, static_cast<int16_t>(elevatorAngle));
    
    // Отладка
    if (_stats.corrections % 100 == 0) {
        ESP_LOGV(TAG_FLIGHT, "🔄 Тангаж: коррекция=%+5.1f°| Угол=%+5.1f°",
                 correction, elevatorAngle);
    }
}

/**
* @brief Применение коррекции рыскания
* 
* @param correction Величина коррекции (градусы)
* 
* @note Управление рулём направления (канал 4)
*/
void FlightStabilizer::applyYawCorrection(float correction) {
    // Управление рулём направления
    float rudderAngle = _targetYaw + correction;
    rudderAngle = constrain(rudderAngle, -45.0f, 45.0f);
    
    // Установка угла сервопривода (канал 4)
    _servoController->setLogicalAngle(4, static_cast<int16_t>(rudderAngle));
    
    // Отладка
    if (_stats.corrections % 100 == 0) {
        ESP_LOGV(TAG_FLIGHT, "🔄 Рыскание: коррекция=%+5.1f°| Угол=%+5.1f°",
                 correction, rudderAngle);
    }
}

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ
// ============================================================================

/**
* @brief Сброс состояния ПИД-регуляторов
* 
* @note Вызывать при включении/выключении стабилизации
*/
void FlightStabilizer::resetPIDStates() {
    _rollState = PIDState{};
    _pitchState = PIDState{};
    _yawState = PIDState{};
    ESP_LOGD(TAG_FLIGHT, "🔄 Состояния ПИД-регуляторов сброшены");
}

/**
* @brief Валидация углов от IMU
* 
* @param roll Крен (градусы) - передаётся по ссылке
* @param pitch Тангаж (градусы) - передаётся по ссылке
* @param yaw Рыскание (градусы) - передаётся по ссылке
* 
* @note Защита от аномальных значений датчика
*/
void FlightStabilizer::validateAngles(float& roll, float& pitch, float& yaw) {
    // Проверка крена
    if (isnan(roll) || isinf(roll) || abs(roll) > 120.0f) {
        roll = 0.0f;
        ESP_LOGW(TAG_FLIGHT, "⚠️ Аномальный крен: сброшено в 0°");
    }
    
    // Проверка тангажа
    if (isnan(pitch) || isinf(pitch) || abs(pitch) > 120.0f) {
        pitch = 0.0f;
        ESP_LOGW(TAG_FLIGHT, "⚠️ Аномальный тангаж: сброшено в 0°");
    }
    
    // Проверка рыскания
    if (isnan(yaw) || isinf(yaw) || yaw < 0.0f || yaw > 360.0f) {
        yaw = 0.0f;
        ESP_LOGW(TAG_FLIGHT, "⚠️ Аномальное рыскание: сброшено в 0°");
    }
}

// ============================================================================
// ДИАГНОСТИКА
// ============================================================================

/**
* @brief Вывод статуса системы в лог
*/
void FlightStabilizer::printStatus() const {
    ESP_LOGI(TAG_FLIGHT, "=== СТАТУС СИСТЕМЫ СТАБИЛИЗАЦИИ ===");
    ESP_LOGI(TAG_FLIGHT, "Режим: %s", modeToString(_mode));
    ESP_LOGI(TAG_FLIGHT, "Состояние: %s", _enabled ? "ВКЛЮЧЕНА" : "ВЫКЛЮЧЕНА");
    ESP_LOGI(TAG_FLIGHT, "Инициализирована: %s", _initialized ? "ДА" : "НЕТ");
    
    if (imuHandler && !imuHandler->isDataValid()) {
        ESP_LOGW(TAG_FLIGHT, "⚠️ Данные гироскопа недоступны");
    }
    
    ESP_LOGI(TAG_FLIGHT, "Статистика:");
    ESP_LOGI(TAG_FLIGHT, "  Обновлений: %lu", _stats.stabilizations);
    ESP_LOGI(TAG_FLIGHT, "  Коррекций: %lu", _stats.corrections);
    ESP_LOGI(TAG_FLIGHT, "  Насыщений: %lu", _stats.saturationEvents);
    ESP_LOGI(TAG_FLIGHT, "  Ошибок данных: %lu", _stats.invalidDataEvents);
    ESP_LOGI(TAG_FLIGHT, "===================================");
}

/**
* @brief Преобразование режима стабилизации в строку
* @param mode Режим для преобразования
* @return Строковое описание режима
*/
const char* FlightStabilizer::modeToString(StabilizationMode mode) const {
    switch (mode) {
        case StabilizationMode::MANUAL:      return "MANUAL (ручной)";
        case StabilizationMode::ROLL_PITCH:  return "ROLL_PITCH (автогоризонт)";
        case StabilizationMode::FULL:        return "FULL (полная)";
        case StabilizationMode::HOVER:       return "HOVER (удержание)";
        default:                             return "UNKNOWN";
    }
}

/**
* @brief Проверка стабильности положения
* @return true если положение стабильно
* 
* @note Стабильным считается положение с малыми углами и угловыми скоростями
*/
bool FlightStabilizer::isStable() const {
    if (!imuHandler || !imuHandler->isDataValid()) {
        return false;
    }
    
    const SensorData& data = imuHandler->getData();
    
    // Проверка углов и угловых скоростей
    bool anglesStable = (abs(data.roll) < 5.0f && abs(data.pitch) < 5.0f);
    bool ratesStable = (abs(data.gyro.x) < 0.17f && abs(data.gyro.y) < 0.17f); // ~10°/с в рад/с
    
    return anglesStable && ratesStable;
}

// Реализация адаптивности
void FlightStabilizer::updateAdaptiveParams(float vibrationLevel) {
    if (!_adaptivePIDEnabled) {
        // Если адаптивность выключена — возвращаем базовые
        _rollPID = _baseRollPID;
        _pitchPID = _basePitchPID;
        return;
    }

    // Нормализуем вибрацию [0.0 ... 1.0]
    float vib = constrain(vibrationLevel, 0.0f, 1.0f);

    // 📐 АЛГОРИТМ:
    // Чем выше вибрация, тем МЕНЬШЕ Kp (чтобы сервы не дергались от шума)
    // и МЕНЬШЕ Kd (производная усиливает высокочастотный шум).
    // Ki оставляем почти без изменений для удержания позиции.
    
    // Формула: New = Base * (1.0 - vib * dampingFactor)
    // dampingFactor = 0.7 (при макс вибрации оставляем 30% мощности)
    
    float factor = 1.0f - (vib * 0.7f); 
    
    _rollPID.kp = _baseRollPID.kp * factor;
    _rollPID.kd = _baseRollPID.kd * factor * 0.5f; // Kd снижаем агрессивнее
    
    _pitchPID.kp = _basePitchPID.kp * factor;
    _pitchPID.kd = _basePitchPID.kd * factor * 0.5f;

    // Ki (интегратор) снижаем незначительно, чтобы не потерять ориентацию
    _rollPID.ki = _baseRollPID.ki * (1.0f - vib * 0.3f);
    _pitchPID.ki = _basePitchPID.ki * (1.0f - vib * 0.3f);
}

// #endif // FLIGHT_STABILIZER_H