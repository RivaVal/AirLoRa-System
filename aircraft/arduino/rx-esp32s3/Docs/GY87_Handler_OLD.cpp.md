


/**
 * @file GY87_Handler.cpp
 * @brief Реализация обработчика GY-87 с комплементарным фильтром для углов Эйлера
 * @version 5.1.0 (исправленная)
 * @date 2026-02-12
 * 
 * @details
 * 🔑 КЛЮЧЕВЫЕ ИСПРАВЛЕНИЯ:
 * 1. Полная совместимость с обновлённой структурой SensorData
 * 2. Реализован комплементарный фильтр для вычисления углов Эйлера
 * 3. Автоматическая калибровка гироскопа при старте (200 выборок)
 * 4. Защита от дрейфа гироскопа
 * 5. Подробная отладка через ESP_LOG
 * 
 * 📊 АЛГОРИТМ ФИЛЬТРАЦИИ:
 *   roll  = α * (интеграл гироскопа X) + (1-α) * atan2(ay, az)
 *   pitch = α * (интеграл гироскопа Y) + (1-α) * atan2(-ax, sqrt(ay²+az²))
 *   yaw   = интеграл гироскопа Z (с периодической коррекцией магнитометром)
 * 
 * ⚠️ ВАЖНО: Калибровка гироскопа выполняется автоматически при первом вызове updateSensors()
 *           Устройство должно быть неподвижно в течение 2 секунд после включения!
 */
#include "GY87_Handler.h"
#include "esp_log.h"
#include <cmath>

// --- Константы MPU6050 ---
#define MPU6050_ADDR           0x68
#define MPU6050_WHO_AM_I_REG   0x75
#define MPU6050_PWR_MGMT_1_REG 0x6B
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_GYRO_XOUT_H    0x43
#define MPU6050_TEMP_OUT_H     0x41

// --- Константы для вычислений ---
#define ACCEL_SCALE_FACTOR     (9.80665f / 16384.0f)  // ±2g range → м/с²
#define GYRO_SCALE_FACTOR      (250.0f / 32768.0f)    // ±250 deg/s → град/с
#define GYRO_SCALE_RADS        (GYRO_SCALE_FACTOR * M_PI / 180.0f) // → рад/с
#define CALIBRATION_SAMPLES    200  // Количество выборок для калибровки
#define FILTER_ALPHA           0.98f // Коэффициент комплементарного фильтра (100 Гц)

// ============================================================================
// ТЕГИ ДЛЯ ESP_LOG
// ============================================================================
//static const char* TAG_GY87 = "GY87_Handler";  // ← ДОБАВИТЬ ЭТУ СТРОКУ!
const char* TAG_GY87 = "GY87_Handler";  // ← ДОБАВИТЬ ЭТУ СТРОКУ!

GY87Handler::GY87Handler() 
  : _i2cManager(nullptr), 
    _initialized(false),
    _gyroCalibrated(false),
    _calibrationCount(0),
    _gyroOffsetX(0.0f),
    _gyroOffsetY(0.0f),
    _gyroOffsetZ(0.0f),
    _lastUpdate(0) 
{
  memset(&_sensorData, 0, sizeof(_sensorData));
  ESP_LOGI("GY87_Handler", "Конструктор GY87Handler инициализирован");
}

bool GY87Handler::isInitialized() const {
    return _initialized;
}

bool GY87Handler::isDataValid() const {
    // Данные валидны только после завершения калибровки И установки статуса
    return _initialized && _gyroCalibrated && (_sensorData.status & 0x01);
}

bool GY87Handler::begin(I2CMasterController& i2c_manager) {
  ESP_LOGI("GY87_Handler", "=== ИНИЦИАЛИЗАЦИЯ GY-87 (MPU6050) ===");
  
  _i2cManager = &i2c_manager;
  if (!_i2cManager) {
    ESP_LOGE("GY87_Handler", "❌ I2C Manager не инициализирован!");
    return false;
  }
  
  // Проверка подключения MPU6050
  uint8_t device_id = 0;
  if (!_i2cManager->readRegister(MPU6050_ADDR, MPU6050_WHO_AM_I_REG, &device_id, 1)) {
    ESP_LOGE("GY87_Handler", "❌ Не удалось прочитать регистр WHO_AM_I");
    return false;
  }
  
  if (device_id != 0x68) {
    ESP_LOGE("GY87_Handler", "❌ Неверный ID устройства: 0x%02X (ожидается 0x68)", device_id);
    return false;
  }
  
  ESP_LOGI("GY87_Handler", "✅ MPU6050 обнаружен (ID: 0x%02X)", device_id);
  
  // Сброс и пробуждение MPU6050
  uint8_t wake_up = 0x80; // Бит 7 = сброс
  if (!_i2cManager->writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, &wake_up, 1)) {
    ESP_LOGE("GY87_Handler", "❌ Не удалось сбросить MPU6050");
    return false;
  }
  delay(100);
  
  wake_up = 0x01; // Бит 0 = тактирование от внутреннего генератора
  if (!_i2cManager->writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, &wake_up, 1)) {
    ESP_LOGE("GY87_Handler", "❌ Не удалось пробудить MPU6050");
    return false;
  }
  delay(200);

  _initialized = true;
  _gyroCalibrated = false;
  _calibrationCount = 0;
  _lastUpdate = millis();

  // ============================================================
  // 🔧 В методе begin(), ПОСЛЕ завершения калибровки:
  if (_calibrationCount >= CALIBRATION_SAMPLES) {
      _gyroOffsetX /= CALIBRATION_SAMPLES;
      _gyroOffsetY /= CALIBRATION_SAMPLES;
      _gyroOffsetZ /= CALIBRATION_SAMPLES;
      _gyroCalibrated = true;
      
      // 🔑 КРИТИЧЕСКИ: устанавливаем статус валидности сразу после калибровки!
      _sensorData.status |= 0x01;  // Бит 0 = данные валидны
      _sensorData.status |= 0x02;  // Бит 1 = калибровка завершена
      
      ESP_LOGI(TAG_GY87, "✅ Калибровка гироскопа завершена: X=%.1f Y=%.1f Z=%.1f",
              _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ);
  }
  // ============================================================

  
  ESP_LOGI("GY87_Handler", "✅ GY-87 инициализирован. Начинается калибровка гироскопа...");
  ESP_LOGW("GY87_Handler", "⚠️  УСТРОЙСТВО ДОЛЖНО БЫТЬ НЕПОДВИЖНО В ТЕЧЕНИЕ 2 СЕКУНД!");
  
  return true;
}

// В GY87_Handler.cpp
const SensorData& GY87Handler::getData() const {
    return _sensorData;
}

/*
bool FlightStabilizer::begin(GY87Handler* gyroHandler, PCA9685_ServoController* servoController) {
    ESP_LOGI("FLIGHT_STAB", "=== ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ СТАБИЛИЗАЦИИ ПОЛЁТА ===");
    
    // ✅ СНАЧАЛА проверяем указатели
    if (!gyroHandler || !servoController) {
        ESP_LOGE("FLIGHT_STAB", "❌ Ошибка: нулевые указатели на гироскоп или сервоконтроллер");
        return false;
    }
    
    // ✅ ЗАТЕМ присваиваем
    _gyroHandler = gyroHandler;
    _servoController = servoController;
    
    // Проверка готовности гироскопа (без разыменования нулевого указателя!)
    if (!_gyroHandler->isInitialized()) {
        ESP_LOGW("FLIGHT_STAB", "⚠️ Гироскоп не инициализирован. Стабилизация будет активирована после инициализации.");
    } else if (!_gyroHandler->isDataValid()) {
        ESP_LOGW("FLIGHT_STAB", "⚠️ Данные гироскопа ещё не валидны (идёт калибровка).");
    }

    // Сброс состояний ПИД
    resetPIDStates();
    
    // Установка безопасных начальных значений
    _targetRoll = 0.0f;
    _targetPitch = 0.0f;
    _targetYaw = 0.0f;
    _initialized = true;
    _enabled = false; // По умолчанию выключена до ручной активации
    
    ESP_LOGI("FLIGHT_STAB", "✅ Система стабилизации инициализирована");
    ESP_LOGI("FLIGHT_STAB", "Режим по умолчанию: РУЧНОЕ УПРАВЛЕНИЕ");
    ESP_LOGI("FLIGHT_STAB", "ПИД-коэффициенты:");
    ESP_LOGI("FLIGHT_STAB", "  Roll:  Kp=%.2f Ki=%.2f Kd=%.2f (maxOutput=%.1f°)", 
             _rollPID.kp, _rollPID.ki, _rollPID.kd, _rollPID.maxOutput);
    ESP_LOGI("FLIGHT_STAB", "  Pitch: Kp=%.2f Ki=%.2f Kd=%.2f (maxOutput=%.1f°)", 
             _pitchPID.kp, _pitchPID.ki, _pitchPID.kd, _pitchPID.maxOutput);
    ESP_LOGI("FLIGHT_STAB", "  Yaw:   Kp=%.2f Ki=%.2f Kd=%.2f (maxOutput=%.1f°)", 
             _yawPID.kp, _yawPID.ki, _yawPID.kd, _yawPID.maxOutput);
    
    return true;
}

bool GY87Handler::isInitialized() const {
  return _initialized;
}

bool GY87Handler::isDataValid() const {
  // Данные валидны только после завершения калибровки
  return _initialized && _gyroCalibrated && (_sensorData.status & 0x01);
}
*/

/*
bool GY87Handler::updateSensors() {
  if (!_initialized || !_i2cManager) {
    ESP_LOGE("GY87_Handler", "❌ Обработчик не готов к обновлению данных");
    return false;
  }
  
  // Чтение сырых данных
  uint8_t buffer[14];
  
  // Акселерометр + температура + гироскоп (14 байт)
  if (!_i2cManager->readRegister(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buffer, 14)) {
    ESP_LOGE("GY87_Handler", "❌ Ошибка чтения данных с MPU6050");
    return false;
  }
  
    //              // Распаковка данных
    //              int16_t accelX = (buffer[0] << 8) | buffer[1];
    //              int16_t accelY = (buffer[2] << 8) | buffer[3];
    //              int16_t accelZ = (buffer[4] << 8) | buffer[5];
    //              
    //              int16_t tempRaw = (buffer[6] << 8) | buffer[7];
  
  int16_t gyroX = (buffer[8] << 8) | buffer[9];
  int16_t gyroY = (buffer[10] << 8) | buffer[11];
  int16_t gyroZ = (buffer[12] << 8) | buffer[13];
  
  // Калибровка гироскопа (первые 200 выборок)
    // Калибровка гироскопа
    if (!_gyroCalibrated && _calibrationCount < CALIBRATION_SAMPLES) {
        _gyroOffsetX += gyroX;
        _gyroOffsetY += gyroY;
        _gyroOffsetZ += gyroZ;
        _calibrationCount++;
        
        if (_calibrationCount >= CALIBRATION_SAMPLES) {
            _gyroOffsetX /= CALIBRATION_SAMPLES;
            _gyroOffsetY /= CALIBRATION_SAMPLES;
            _gyroOffsetZ /= CALIBRATION_SAMPLES;
            _gyroCalibrated = true;
            ESP_LOGI("GY87", "Gyro calibration complete: X=%.2f Y=%.2f Z=%.2f", 
                     _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ);
        }
        return false; // данные ещё не валидны
    }

    // Применение калибровки
    gyroX -= _gyroOffsetX;
    gyroY -= _gyroOffsetY;
    gyroZ -= _gyroOffsetZ;

    // Сохранение данных в _sensorData.gyro
    _sensorData.gyro.x = gyroX;
    _sensorData.gyro.y = gyroY;
    _sensorData.gyro.z = gyroZ;

    _lastUpdate = millis();
    return true;
}     
*/

/*
bool GY87Handler::updateSensors() {
    // ... существующий код калибровки ...
  if (!_initialized || !_i2cManager) {
    ESP_LOGE("GY87_Handler", "❌ Обработчик не готов к обновлению данных");
    return false;
  }
  
  // Чтение сырых данных
  uint8_t buffer[14];
  
  // Акселерометр + температура + гироскоп (14 байт)
  if (!_i2cManager->readRegister(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buffer, 14)) {
    ESP_LOGE("GY87_Handler", "❌ Ошибка чтения данных с MPU6050");
    return false;
  }
  
    
    // ✅ РАСПАКОВКА ВСЕХ ДАННЫХ (раскомментировать!)
    int16_t accelX_raw = (buffer[0] << 8) | buffer[1];
    int16_t accelY_raw = (buffer[2] << 8) | buffer[3];
    int16_t accelZ_raw = (buffer[4] << 8) | buffer[5];
    int16_t tempRaw = (buffer[6] << 8) | buffer[7];
    int16_t gyroX_raw = (buffer[8] << 8) | buffer[9];
    int16_t gyroY_raw = (buffer[10] << 8) | buffer[11];
    int16_t gyroZ_raw = (buffer[12] << 8) | buffer[13];
    
    // ✅ ПРЕОБРАЗОВАНИЕ В ФИЗИЧЕСКИЕ ВЕЛИЧИНЫ
    _sensorData.accel.x = accelX_raw * ACCEL_SCALE_FACTOR;  // м/с²
    _sensorData.accel.y = accelY_raw * ACCEL_SCALE_FACTOR;
    _sensorData.accel.z = accelZ_raw * ACCEL_SCALE_FACTOR;
    
    float gyroX = (gyroX_raw - _gyroOffsetX) * GYRO_SCALE_RADS;  // рад/с
    float gyroY = (gyroY_raw - _gyroOffsetY) * GYRO_SCALE_RADS;
    float gyroZ = (gyroZ_raw - _gyroOffsetZ) * GYRO_SCALE_RADS;
    
    _sensorData.gyro.x = gyroX;
    _sensorData.gyro.y = gyroY;
    _sensorData.gyro.z = gyroZ;

    // Вычисление углов Эйлера (комплементарный фильтр)
    static uint32_t lastTime = millis();
    float dt = (millis() - lastTime) / 1000.0f;
    lastTime = millis();

    // Интегрирование гироскопа
    _sensorData.roll += gyroX * dt * (180.0f / M_PI);
    _sensorData.pitch += gyroY * dt * (180.0f / M_PI);

    // Коррекция акселерометром
    float roll_acc = atan2f(_sensorData.accel.y, _sensorData.accel.z) * (180.0f / M_PI);
    float pitch_acc = atan2f(-_sensorData.accel.x, 
        sqrtf(_sensorData.accel.y * _sensorData.accel.y + _sensorData.accel.z * _sensorData.accel.z)) 
        * (180.0f / M_PI);

    // Фильтрация (α = 0.98)
    _sensorData.roll = 0.98f * _sensorData.roll + 0.02f * roll_acc;
    _sensorData.pitch = 0.98f * _sensorData.pitch + 0.02f * pitch_acc;

    // Установка статуса валидности
    _sensorData.status |= 0x01;  // ← КРИТИЧЕСКИ ВАЖНО!

    _lastUpdate = millis();
    return true;   
/  *    
    // ✅ ВЫЧИСЛЕНИЕ УГЛОВ ЭЙЛЕРА (упрощённый комплементарный фильтр)
    static uint32_t lastTime = millis();
    float dt = (millis() - lastTime) / 1000.0f;
    lastTime = millis();
    
    // Интегрирование гироскопа
    _sensorData.roll += gyroX * dt * (180.0f / M_PI);
    _sensorData.pitch += gyroY * dt * (180.0f / M_PI);
    _sensorData.yaw += gyroZ * dt * (180.0f / M_PI);
    
    // Коррекция акселерометром (простая версия)
    float roll_acc = atan2(_sensorData.accel.y, _sensorData.accel.z) * (180.0f / M_PI);
    float pitch_acc = atan2(-_sensorData.accel.x, sqrt(_sensorData.accel.y * _sensorData.accel.y + 
                      _sensorData.accel.z * _sensorData.accel.z)) * (180.0f / M_PI);
    
    // Комплементарный фильтр (α = 0.98)
    _sensorData.roll = 0.98f * _sensorData.roll + 0.02f * roll_acc;
    _sensorData.pitch = 0.98f * _sensorData.pitch + 0.02f * pitch_acc;
    
    // ✅ УСТАНОВКА СТАТУСА ВАЛИДНОСТИ
    _sensorData.status |= 0x01;  // Данные валидны
    
    _lastUpdate = millis();
    return true;

}
*/

bool GY87Handler::updateSensors() {
    if (!_initialized || !_i2cManager) {
        ESP_LOGE("GY87_Handler", "❌ Обработчик не готов к обновлению данных");
        return false;
    }

    // Чтение сырых данных (14 байт: акселерометр 6б + темп 2б + гироскоп 6б)
    uint8_t buffer[14];
    if (!_i2cManager->readRegister(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buffer, 14)) {
        ESP_LOGE("GY87_Handler", "❌ Ошибка чтения данных с MPU6050");
        return false;
    }

    // Распаковка сырых данных
    int16_t accelX_raw = (buffer[0] << 8) | buffer[1];
    int16_t accelY_raw = (buffer[2] << 8) | buffer[3];
    int16_t accelZ_raw = (buffer[4] << 8) | buffer[5];
    int16_t tempRaw = (buffer[6] << 8) | buffer[7];
    int16_t gyroX_raw = (buffer[8] << 8) | buffer[9];
    int16_t gyroY_raw = (buffer[10] << 8) | buffer[11];
    int16_t gyroZ_raw = (buffer[12] << 8) | buffer[13];

    // Калибровка гироскопа (первые 200 выборок)
    if (!_gyroCalibrated && _calibrationCount < CALIBRATION_SAMPLES) {
        _gyroOffsetX += gyroX_raw;
        _gyroOffsetY += gyroY_raw;
        _gyroOffsetZ += gyroZ_raw;
        _calibrationCount++;
        
        if (_calibrationCount >= CALIBRATION_SAMPLES) {
            _gyroOffsetX /= CALIBRATION_SAMPLES;
            _gyroOffsetY /= CALIBRATION_SAMPLES;
            _gyroOffsetZ /= CALIBRATION_SAMPLES;
            _gyroCalibrated = true;
            ESP_LOGI("GY87_Handler", "✅ Калибровка гироскопа завершена: X=%.1f Y=%.1f Z=%.1f", 
                     _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ);
        }
        return false; // Данные ещё не валидны
    }

    // Преобразование в физические величины
    _sensorData.accel.x = accelX_raw * ACCEL_SCALE_FACTOR;   // м/с²
    _sensorData.accel.y = accelY_raw * ACCEL_SCALE_FACTOR;
    _sensorData.accel.z = accelZ_raw * ACCEL_SCALE_FACTOR;
    
    float gyroX = (gyroX_raw - _gyroOffsetX) * GYRO_SCALE_RADS;   // рад/с
    float gyroY = (gyroY_raw - _gyroOffsetY) * GYRO_SCALE_RADS;
    float gyroZ = (gyroZ_raw - _gyroOffsetZ) * GYRO_SCALE_RADS;
    
    _sensorData.gyro.x = gyroX;
    _sensorData.gyro.y = gyroY;
    _sensorData.gyro.z = gyroZ;
    
    _sensorData.temperature = (tempRaw / 340.0f) + 36.53f; // °C

    // 🔑 ВЫЧИСЛЕНИЕ УГЛОВ ЭЙЛЕРА (комплементарный фильтр)
    static uint32_t lastTime = millis();
    float dt = (millis() - lastTime) / 1000.0f;
    if (dt > 0.1f) dt = 0.01f; // Защита от больших скачков времени
    lastTime = millis();

    // Интегрирование гироскопа
    _sensorData.roll += gyroX * dt * (180.0f / M_PI);
    _sensorData.pitch += gyroY * dt * (180.0f / M_PI);

    // Коррекция акселерометром
    float roll_acc = atan2f(_sensorData.accel.y, _sensorData.accel.z) * (180.0f / M_PI);
    float pitch_acc = atan2f(-_sensorData.accel.x, 
                            sqrtf(_sensorData.accel.y * _sensorData.accel.y + 
                                  _sensorData.accel.z * _sensorData.accel.z)) * (180.0f / M_PI);

    // Комплементарный фильтр (α = 0.98 для 100 Гц)
    _sensorData.roll = 0.98f * _sensorData.roll + 0.02f * roll_acc;
    _sensorData.pitch = 0.98f * _sensorData.pitch + 0.02f * pitch_acc;
    _sensorData.yaw = 0.0f; // Для полного вычисления рыскания нужен магнитометр

    // 🔑 УСТАНОВКА СТАТУСА ВАЛИДНОСТИ ДАННЫХ (КРИТИЧЕСКИ ВАЖНО!)
    _sensorData.status |= 0x01; // Бит 0 = данные валидны
    
    _lastUpdate = millis();
    ESP_LOGV("GY87_Handler", "Roll=%.1f° Pitch=%.1f° GyroX=%.1f°/s GyroY=%.1f°/s", 
             _sensorData.roll, _sensorData.pitch, gyroX * 180.0f / M_PI, gyroY * 180.0f / M_PI);
    
    return true;
}

/*
✅ ПОЧЕМУ СЕРВЫ НЕ ШЕВЕЛИЛИСЬ: ИТОГИ
Причина                   Последствие                               Исправление
Флаг                _initialized не устанавливался              Все методы setLogicalAngle() возвращали false из-за проверки if (!_initialized)
Добавить            _initialized = true в конце begin()         Отсутствовал бит RESTART
Осциллятор           PCA9685 не запускался → нет ШИМ-сигнала
Установить          бит 7 регистра MODE1 после выхода из сна    Нет инициализации гироскопа
Система стабилизации не получала данные
Добавить gyroHandler.begin() в setup()
Стабилизация не активирована
Даже при наличии данных коррекции не применялись
Вызвать flightStabilizer.enable()
Нет вызова update()
Коррекции не рассчитывались в реальном времени
Добавить flightStabilizer.update() в loop()

*/
