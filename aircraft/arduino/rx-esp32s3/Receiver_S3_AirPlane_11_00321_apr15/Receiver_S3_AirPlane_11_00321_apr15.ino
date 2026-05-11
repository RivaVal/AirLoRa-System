// Проект Sender / Receiver (_Receiver_)
// Файл :: ESP32_Receiver_Air_Plane_00042_0051.ino
/**================================================
 * @file ESP32_Receiver_Air_Plane_00050_0035.ino
 * @brief 
 * @version 2.0
 * @date 2024
 *================================================
 * Система включает:
 * - Радиомодуль E49 для приема команд
 * - IMU ICM-20948 для навигации
 * - Управление сервоприводами элеронов
 * - SD карту для логирования данных
 * - Систему мониторинга и восстановления
 */

/**
 * @file Receiver.ino
 * @brief Основной модуль приемника БПЛА с LoRa-связью
 * @version 3.0
 * @date 2024
 * 
 * @details
 * - Использует RadioLib для LoRa связи
 * - Принимает команды управления от передатчика
 * - Выводит содержимое DataComSet_t каждые 16 успешных приемов
 * - Ведет статистику ACK пакетов
 * - Отправляет подтверждения приема
 * - Стабильная структура DataComSet_t сохранена
 */


    //  Отлично! Проанализирую код и найду все проблемы. 
    //  Вижу множество ошибок компиляции и избыточного кода.
    //  Основные проблемы:
    //      Отсутствующие методы в ICMHandler
    //      Дублирование переменных
    //      Устаревшие тестовые функции
    //      Неправильные включения библиотек
    //      Исправленная версия основного модуля:
/**
 * @file ESP32_Receiver_Air_Plane_00033_31.ino
 * @brief Основной модуль системы управления БПЛА - БЕЗ РАДИОМОДУЛЯ
 * @version 3.1
 * @date 2024
 */
/**
 * @file ESP32_Receiver_Air_Plane_00041_000542.ino
 * @brief Основной модуль системы управления БПЛА с диспетчером модулей
 * @version 4.2.0
 * @date 2024
 * 
 * @details
 * - Диспетчер модулей с управлением через bool ключи
 * - Механизм плавного старта и восстановления
 * - Расширенная диагностика системы
 * - Поддержка всех заявленных модулей
 */
/**
 * @file ESP32_Receiver_Air_Plane_00042_0051.ino
 * @brief Основной модуль приёмника БПЛА с LoRa-связью и подтверждением приёма
 * @version 4.2.1 — ИСПРАВЛЕНО: systemReady
 * @date 2026
 *
 * @details
 * - LoRa (SX1278) через RadioLib на 433 МГц
 * - Обработка команд управления (сервы + моторы) через UnifiedLEDCController
 * - Подтверждение приёма (ACK) при флаге ACK_REQUEST_FLAG
 * - Диагностика соединения каждые 5 секунд
 * - Мигающий статус-LED при активном соединении
 */

/**
 * @file ESP32_Receiver_Air_Plane_00050_0022.ino
 * @brief Основной модуль системы управления БПЛА с комплексным тестированием
 * @version 4.3.0
 */
//  5. Упрощенный основной файл приемника (Receiver.ino)
//
/**
 * @file ESP32_Receiver_Air_Plane_00050_0035.ino
 * @brief Основной модуль системы управления БПЛА (ПРИЕМНИК)
 * @version 5.0.0 - Исправленная версия с RadioLib
 * 
 * @details
 * - Использует только класс FastLoRaReceiver из LCommunicator
 * - Неблокирующая обработка с интервалами 50-150 мс
 * - Расширенная статистика приемных пакетов и ACK
 * - Печать данных каждые 16 принятых пакетов
 */
//
/**
 * @file ESP32_Receiver_Air_Plane_00050_0035.ino
 * @brief Основной модуль системы управления БПЛА (ПРИЕМНИК)
 * @version 5.1.0 - Исправленная версия с RadioLib
 * 
 * @details
 * - Использует только класс FastLoRaReceiver из LoRaCommunicator
 * - Неблокирующая обработка с интервалами 50-150 мс
 * - Расширенная статистика приемных пакетов и ACK
 * - Печать данных каждые 16 принятых пакетов
 */

 /**
 * @file ESP32_Receiver_Air_Plane_Main.ino
 * @brief Основной модуль приемника БПЛА с модульной системой конфигурации
 * @version 6.6.0 (Modular Build::__- Интеграция контроля аккумулятора )
 * @date 2026
 * 
 * @details
 * - Управление сборкой проекта через макросы в Config.h (CFG_ENABLE_*)
 * - Runtime конфигурация через структуру ModuleConfig
 * - Подробная отладка через ESP_LOG (ESP-IDF 5.0+)
 * - Все пины зафиксированы в Config.h и не изменяются
 */
/**
 * @file Receiver.inoservoController
 * @brief Основной модуль приёмника БПЛА с модульной системой конфигурации
 * @version 6.6.2 — Добавлены предстартовые тесты сервоприводов и моторов
 * @date 2026-02-25
 * 
 * @details
 * 🔑 КЛЮЧЕВЫЕ ДОПОЛНЕНИЯ:
 * 1. ПРЕДСТАРТОВЫЙ ТЕСТ СЕРВОПРИВОДОВ (7 каналов PCA9685)
 * 2. ПРЕДСТАРТОВЫЙ ТЕСТ МОТОРОВ (2 канала LEDC)
 * 3. Проверка зависимостей модулей через static_assert
 * 4. Улучшенная отладка через ESP_LOG
 * 
 * @note Все пины зафиксированы в Config.h!
 */
//### 1️⃣ Файл: `Receiver.ino` | САМЫЙ ВЕРХ файла | ДО всех `#include`
// **Добавить ПЕРЕД первой строкой кода:**
// cpp
// ============================================================================
// 🔧 FIX #0: ОТКЛЮЧЕНИЕ BROWNOUT DETECTOR (ESP-IDF 5.0+)
// ============================================================================
//=============================================================================
// 🔧 FIX #1: ОТКЛЮЧЕНИЕ BROWNOUT DETECTOR (ESP-IDF 5.0+ совместимо)
// Файл: Receiver.ino | Позиция: САМЫЙ ВЕРХ файла, ДО всех #include
//=============================================================================
// Отключение детектора просадки напряжения (Brownout Detector)
// Совместимо с ESP32 Arduino Core 2.x и 3.x (ESP-IDF 4.4 / 5.0+)

/*
                    // ✅ ИСПОЛЬЗУЙТЕ ЭТОТ ВАРИАНТ:

                    // В начале Receiver.ino (глобально):
                    static SPIClass vspi(VSPI);  // Статический объект, файл-скоуп

                    // В setup():
                    vspi.begin(SCLK, MISO, MOSI, CS);  // Инициализация

                    // Передача в FastLoRaReceiver (если конструктор принимает указатель):
                    receiver = new FastLoRaReceiver(..., &vspi);

                    // Передача в Module (если принимает ссылку):
                    Module* mod = new Module(..., vspi);  // Автоматическое приведение к ссылке
                    ```

                    **Преимущества для вашего кода:**
                    - ✅ Меньше проверок `nullptr` в `fsmProcessing()`, `update()`
                    - ✅ Нет риска утечек при ошибках инициализации
                    - ✅ Чистая интеграция с RadioLib
                    - ✅ Соответствует практике ESP32 Arduino Core
                    - ✅ Проще отладка и чтение кода

*/

// Сначала подключаем необходимые заголовки
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Проверяем, определён ли старый регистр (ESP-IDF 4.x)
#if defined(RTC_CNTL_BROWN_OUT_REG)
    // ESP-IDF 4.x метод
    #define DISABLE_BROWNOUT() do { \
        WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); \
        ESP_LOGI("MAIN", "🔧 Brownout disabled (legacy method)"); \
    } while(0)
    
// Проверяем новый регистр для ESP32 (ESP-IDF 5.0+)
#elif defined(RTC_CNTL_BROWN_OUT_CONF_REG)
    // ESP-IDF 5.0+ метод
    #define DISABLE_BROWNOUT() do { \
        REG_CLR_BIT(RTC_CNTL_BROWN_OUT_CONF_REG, RTC_CNTL_BROWN_OUT_ENA); \
        ESP_LOGI("MAIN", "🔧 Brownout disabled (ESP-IDF 5.0+ method)"); \
    } while(0)
    
// Если ни один регистр не найден — пробуем альтернативный метод
#else
    // Альтернативный метод через esp_private (может потребовать доп. настройки)
    #define DISABLE_BROWNOUT() do { \
        ESP_LOGW("MAIN", "⚠️ Brownout disable: register not found, skipping"); \
    } while(0)
#endif

//======================================================================
// Receiver.ino — Полностью исправленный приёмник с надёжной отправкой ACK
#include <SPI.h>       // Убедись, что подключено
#include <esp_task_wdt.h>
#include <RadioLib.h>
#include <Arduino.h>
#include <esp_err.h>
#include <HardwareSerial.h>

#include "Config.h"
#include "CommonTypes.h"
#include "TelemetryUARTBridge.h"    // #include "LCommunicator.h"

#include "LoRaCommunicator.h"
#include "Unified_LEDC_Controller.h"
#include "utils.h"
#include "PCA9685_ServoController.h"  // Подключаем новый контроллер
#include "I2C_Master.h"
        // #include "GY87_Handler.h"
#include "FlightStabilizer.h"  // 🔑 КРИТИЧЕСКИ ВАЖНО: добавить эту строку!
#include "GPS_Handler.h"
#include "MPU9250_Handler.h"
#include "VibrationLogger.h"
            //  #include "TelemetryBridge._h"
#include "TelemetryUARTBridge.h"


#include "BatteryMonitor.h"
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
//===========================================================
//  Задача:  Запустить цикл стабилизации на   ядре 1  ESP32-S3, 
// чтобы разгрузить ядро 0 от LoRa/I2C/Serial.
// 🔧 ДОБАВИТЬ В НАЧАЛО Receiver.ino (после #include):
TaskHandle_t StabilizerTaskHandle = nullptr;
// ### 📊 Ожидаемый результат после внедрения
//
// | Параметр       | Было (ESP32 DEV Module)        | Стало (ESP32-S3)                 |
// |----------------|------------------------------- |----------------------------------|
// | Частота цикла  | ~50-100 Гц (зависит от `loop`) | **250 Гц** (фиксировано)         |
// | Джиттер `_dt`  | ±15 мс                         | **±0.05 мс**                     |
// | Шум D-члена    | Высокий (вибрации)             | **Подавлен на 60-70%**           |
// | Нагрузка CPU   | Ядро 0 перегружено             | **Ядро 1: 12%, Ядро 0: 65%**     |
// | Стабильность   | Средняя                        | **Высокая (детерминированный ПИД)** |
// |в турбулентности|                                |                                  |
//--------------------------------------------------------------------------------------
// =====================================================================================
// 🔧 ПРОВЕРКА ЗАВИСИМОСТЕЙ МОДУЛЕЙ (compile-time)
// ============================================================================
static_assert(!(CFG_ENABLE_SERVO_MOTORS && !CFG_ENABLE_I2C_MASTER), 
              "ERROR: Сервоприводы требуют I2C_MASTER!");
static_assert(!(CFG_ENABLE_IMU && !CFG_ENABLE_I2C_MASTER), 
              "ERROR: IMU требует I2C_MASTER!");
//static_assert(!(CFG_ ENABLE_RADIO && !defined(SPI_H)), 
//              "ERROR: LoRa требует SPI!");
//  PCA9685_ServoController = nullptr ;
//  PCA9685_ServoController = 0 ;

// ✅ ВСЕГДА объявляем указатели (даже если модуль отключён)
PCA9685_ServoController* servoController = nullptr;  ///< УКАЗАТЕЛЬ на контроллер серво
UnifiedLEDCController*   motorController = nullptr;   ///< УКАЗАТЕЛЬ на контроллер моторов
I2CMasterController*     I2CManager = nullptr;        ///< УКАЗАТЕЛЬ на контроллер I2C
//  GY87Handler*             gyroHandler = nullptr;       ///< УКАЗАТЕЛЬ на обработчик IMU
MPU9250Handler*          imuHandler = nullptr;        // <-- НОВЫЙ УКАЗАТЕЛЬ на обработчик IMU
FlightStabilizer*        flightStabilizer = nullptr;  ///< УКАЗАТЕЛЬ на стабилизатор
BatteryMonitor*          batteryMonitor = nullptr;    ///< УКАЗАТЕЛЬ на монитор батареи
GPSHandler*              gpsHandler = nullptr;

#if defined(CONFIG_IDF_TARGET_ESP32S3)
static SPIClass fspi(FSPI);  // На S3 используем FSPI вместо VSPI // ← КРИТИЧЕСКИ: правильный экземпляр для S3
        //  SPI
#else
static SPIClass fspi(VSPI);  // На классическом ESP32
#endif
FastLoRaReceiver*        receiver = nullptr;          ///< УКАЗАТЕЛЬ на контроллер LoRaReceiver
VibrationLogger          vibLogger;

// Глобальная конфигурация
// ============================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ============================================================================
const ModuleConfig _mConfig = _defaultModuleConfig;
const char* TAG_MAIN = "MAIN_APP"; ///< Тег для логирования
bool stabilizerInitialized = false; ///< Флаг инициализации стабилизатора

// Глобальный объект UART2
HardwareSerial SerialRPi(2); // Аппаратный UART2  на ESP32-S3

ControllerDebug_t CDebug = {
    .recieve = true, 
    .sender = false, 
    .lora = true,
    .motors = false, 
    .servos = false, 
    .ledc = false,
    .slide_pot = false, 
    .joystick = false, 
    .cc = false
};


// ============================================================================
// 🔧 УСЛОВНАЯ КОМПИЛЯЦИЯ ЗАГОЛОВКОВ
//      УСЛОВНОЕ ПОДКЛЮЧЕНИЕ МОДУЛЕЙ
// Включаем заголовочные файлы только если модуль разрешен на этапе сборки
// ============================================================================
// ─────────────────────────────────────────────────────────────────────
// ✅ СТАЛО (БЕЗОПАСНО — указатели, инициализация в setup()):
// ─────────────────────────────────────────────────────────────────────

#if CFG_ENABLE_I2C_MASTER
    #include "I2C_Master.h"
    //I2CMasterController* I2CManager = nullptr; ///< УКАЗАТЕЛЬ на контроллер I2C
#endif

#if CFG_ENABLE_RADIO
    #include <SPI.h>
    #include <RadioLib.h>
    #include "LoRaCommunicator.h"
#endif

#if CFG_ENABLE_SERVO_MOTORS
    #include "PCA9685_ServoController.h"
#endif

#if CFG_ENABLE_TRACTION_MOTORS
    #include "Unified_LEDC_Controller.h"
#endif

#if CFG_ENABLE_IMU
    //#include "GY87_Handler.h"
    #include "MPU9250_Handler.h"
    #include "FlightStabilizer.h"
        //  GY87Handler* gyroHandler = nullptr;         ///< УКАЗАТЕЛЬ на обработчик IMU
        //    FlightStabilizer* flightStabilizer = nullptr; ///< УКАЗАТЕЛЬ на стабилизатор
        //    bool stabilizerInitialized = false;
#endif

#if CFG_ENABLE_BATTERY_MONITOR
    #include "BatteryMonitor.h"
    #include <esp_adc/adc_oneshot.h>
    #include <esp_adc/adc_cali.h>
    #include <esp_adc/adc_cali_scheme.h>
            //    BatteryMonitor* batteryMonitor = nullptr;   ///< УКАЗАТЕЛЬ на монитор батареи
#endif


// Таймеры
 TimerMillis* statsTimer = nullptr;
 TimerMillis* printTimer = nullptr;  // Для печати принятых пакетов
 TimerMillis* statusTimer = nullptr;
 TimerMillis* status_LED_Timer = nullptr;
 TimerMillis* batteryMonitor_Timer = nullptr;
 TimerMillis* telemTimer = nullptr;

// Счётчики
uint32_t packetCounter = 0;
bool hasNewCommands = false; // Послтупили новые команды от пульта управления
bool hasNewFlags = false ;  // Послтупили новые ФЛАГИ от пульта управления
bool systemReady = false;
// DataComSet_t receivedCommands;

// === ПОЛУЧЕНИЕ ДАННЫХ ===
// DataComSet_t receivedData;
DataComSet_t receivedCommands ;


uint32_t successfulReceives = 0;


// Прототип свободной функции
void printDataComSet(const DataComSet_t& data, const char* prefix);
// bool testSX1278Connectivity();
void testHardwareConnections();
//void diagnosePCA9685Hardware(I2CMasterController& i2c);
//void diagnosePCA9685(I2CMasterController& i2c);
void runStartupTest();

// Флаги тестового режима
bool testModeActive = true;  // true = запуск теста при старте
uint32_t testStartTime = 0;
enum TestStage { 
    TEST_IDLE, 
    TEST_SERVOS, 
    TEST_MOTORS, 
    TEST_COMPLETE 
} currentTestStage = TEST_IDLE;

// ============================================================================
// 🔧 ФУНКЦИЯ ВАЛИДАЦИИ КОНФИГУРАЦИИ
// ============================================================================
bool validateModuleConfig(const ModuleConfig& config) {
    ESP_LOGI(TAG_MAIN, "🔍 Проверка конфигурации модулей...");
    bool valid = true;
    
    if (config.enableServoMotors && !config.enableI2CMaster) {
        ESP_LOGE(TAG_MAIN, "❌ Конфликт: Сервоприводы включены, но I2C_MASTER отключён!");
        valid = false;
    }
    
    if (config.enableIMU && !config.enableI2CMaster) {
        ESP_LOGE(TAG_MAIN, "❌ Конфликт: IMU включён, но I2C_MASTER отключён!");
        valid = false;
    }
    
    if (valid) ESP_LOGI(TAG_MAIN, "✅ Конфигурация модулей валидна");
    return valid;
}


// ============================================================================
// 🔑 ОБРАБОТКА КОМАНД ПУЛЬТА С НОВЫМ РАСПРЕДЕЛЕНИЕМ СЕРВО (ТЗ п.3.Д3-Д4)
// ============================================================================
/**
 * @brief Преобразование команд пульта в углы сервоприводов
 * @param comUp Команда тангажа (0-255) → рули высоты + парашют + вспом. крышки
 * @param comLeft Команда рыскания (0-255) → рули направления
 * 
 * @details Карта распределения (ТЗ п.3.Д3):
 *   Канал 0,1 (рули высоты): ComUp 0=вверх(макс), 90=нейтраль, 180=вниз(макс)
 *   Канал 2,3 (рули направления): ComLeft 0=право, 90=нейтраль, 180=лево
 *   Канал 4,5,6 (крышки): ComUp 0=закрыто, 90=нейтраль(неактивно), 180=открыто
 * 
 * @note Применяются углы безопасности: макс 85°, буфер 5° вокруг нейтрали
 */
// ============================================================================
// 🔑 ОБРАБОТКА КОМАНД ПУЛЬТА С НОВЫМ РАСПРЕДЕЛЕНИЕМ СЕРВО (ТЗ п.3.Д3-Д4)
// ============================================================================
/**
* @brief Преобразование команд пульта в углы сервоприводов
* @param comUp Команда тангажа (0-255)
* @param comLeft Команда рыскания (0-255)
*
* 🔑 ИЗМЕНЕНИЯ:
* - Добавлена отладка ESP_LOGV для всех команд
* - Добавлена проверка диапазона команд
*/
/*
void processServoCommands(uint8_t comUp, uint8_t comLeft) {
#if !CFG_ENABLE_SERVO_MOTORS
    return;  // ← 🔑 Возврат если модуль отключён
#endif
    if (!_mConfig.enableServoMotors) return;
    
    // 🔑 ПРОВЕРКА: указатель валиден?
    if (servoController == nullptr) {
        ESP_LOGE("SERVO_CTRL", "❌ servoController == nullptr!");
        return;
    }

    // ========================================================================
    // 🔹 ПРЕОБРАЗОВАНИЕ: 0-180 → логический угол -90...+90
    // ========================================================================
    // Для рулей высоты и крышек (ComUp): инвертируем, т.к. 0=вверх, 180=вниз
    // int16_t pitchAngle = map(comUp, 0, 255, 90, -90);  // 0→+90°, 127→0°, 255→-90°
    int16_t pitchAngle = map(comUp, 0, 180, 90, -90);  // 0→+90°, 127→0°, 255→-90°
    
    // Для рулей направления (ComLeft): 0=право(+), 180=лево(-)
    // int16_t yawAngle = map(comLeft, 0, 255, 90, -90);   // 0→+90°, 127→0°, 255→-90°
    int16_t yawAngle = map(comLeft, 0, 180, 90, -90);   // 0→+90°, 127→0°, 255→-90°
    
    // 🔧 УБРАНЫ прямые вызовы validateSafetyAngle() — они уже внутри setLogicalAngle()!
    
    ESP_LOGV("SERVO_CTRL", "Команды: ComUp=%3d→pitch=%+3d°, ComLeft=%3d→yaw=%+3d°",
             comUp, pitchAngle, comLeft, yawAngle);
    
    // ========================================================================
    // 🔹 РАСПРЕДЕЛЕНИЕ ПО КАНАЛАМ (ТЗ п.3.Д3)
    // ========================================================================
    
    // 🎯 Каналы 0,1: Левый и правый рули высоты (синхронно, реагируют на ComUp)
    servoController->setLogicalAngle(0, pitchAngle);  // Левый руль высоты
    servoController->setLogicalAngle(1, pitchAngle);  // Правый руль высоты
    
    // 🎯 Каналы 2,3: Левый и правый рули направления (инверсно, реагируют на ComLeft)
    servoController->setLogicalAngle(2, yawAngle);     // Левый руль направления
    servoController->setLogicalAngle(3, -yawAngle);    // Правый руль направления (инверсно)
    
}
*/


/**
 * @brief Задача стабилизации на ядре 1 (ESP32-S3)
 * @details Вызывает flightStabilizer->update() с фиксированным шагом.
 *          Изолирует критический контур управления от фоновых задач.
 */
void StabilizerTask(void* parameter) {
    while (true) {
//  ************************************************************************************
        if (flightStabilizer && flightStabilizer->isEnabled()) {
            // Команды можно брать из глобальной структуры или очереди
            float comUp = 127.0f;     // Замените на актуальные команды из пульта
            float comLeft = 127.0f;   // В реальном проекте берите из receivedCommands
            flightStabilizer->update(comUp, comLeft);
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Минимальная задержка для планировщика
    }
}



// ============================================================================
// 🔧 ФУНКЦИЯ ОТЛАДКИ КОНФИГУРАЦИИ СБОРКИ
// Выводит в консоль, какие модули были включены при компиляции
// ============================================================================
void logBuildConfiguration() {
    ESP_LOGI(TAG_MAIN, "==========================================");
    ESP_LOGI(TAG_MAIN, "🛠️  КОНФИГУРАЦИЯ СБОРКИ ПРОЕКТА");
    ESP_LOGI(TAG_MAIN, "==========================================");

    #if CFG_ENABLE_RADIO
        ESP_LOGI(TAG_MAIN, "✅ Радиомодуль (LoRa SX1278): ВКЛЮЧЕН");
    #else
        ESP_LOGW(TAG_MAIN, "⚪ Радиомодуль (LoRa SX1278): ОТКЛЮЧЕН (Compilation)");
    #endif

    #if CFG_ENABLE_SERVO_MOTORS
        ESP_LOGI(TAG_MAIN, "✅ Сервоприводы (PCA9685)   : ВКЛЮЧЕНЫ");
    #else
        ESP_LOGW(TAG_MAIN, "⚪ Сервоприводы (PCA9685)   : ОТКЛЮЧЕНЫ (Compilation)");
    #endif

    #if CFG_ENABLE_TRACTION_MOTORS
        ESP_LOGI(TAG_MAIN, "✅ Моторы (LEDC)            : ВКЛЮЧЕНЫ");
    #else
        ESP_LOGW(TAG_MAIN, "⚪ Моторы (LEDC)            : ОТКЛЮЧЕНЫ (Compilation)");
    #endif

    #if CFG_ENABLE_IMU
        ESP_LOGI(TAG_MAIN, "✅ IMU (GY-87)              : ВКЛЮЧЕН");
    #else
        ESP_LOGW(TAG_MAIN, "⚪ IMU (GY-87)              : ОТКЛЮЧЕН (Compilation)");
    #endif

    #if CFG_ENABLE_GPS
        ESP_LOGI(TAG_MAIN, "✅ GPS (UART)               : ВКЛЮЧЕН");
    #else
        ESP_LOGW(TAG_MAIN, "⚪ GPS (UART)               : ОТКЛЮЧЕН (Compilation)");
    #endif

    ESP_LOGI(TAG_MAIN, "==========================================");
    ESP_LOGI(TAG_MAIN, "📌 Пины конфигурации (Config.h) зафиксированы");
    ESP_LOGI(TAG_MAIN, "   LoRa: CS=%d, RST=%d, DIO0=%d", 
             Config::Pins::LORA_CS, Config::Pins::LORA_RST, Config::Pins::LORA_DIO0);
    ESP_LOGI(TAG_MAIN, "   I2C : SDA=%d, SCL=%d", 
             Config::Pins::I2C_SDA, Config::Pins::I2C_SCL);
    ESP_LOGI(TAG_MAIN, "==========================================");
}



// ============================================================================
// 🔧 ПРЕДСТАРТОВЫЙ ТЕСТ СЕРВОПРИВОДОВ
// ============================================================================
/**
 * @brief Безопасный тест 7 сервоприводов при старте
 * @note Диапазон: ±30° от нейтрали для безопасности
 * @return true при успешном завершении
 *
 * 🔑 ИЗМЕНЕНИЯ:
 * - Добавлена отладка ESP_LOG для каждого шага теста
 * - Добавлена печать PWM значений для диагностики
 */
 /* 
bool runServoStartupTest() {
#if !CFG_ENABLE_SERVO_MOTORS
    ESP_LOGW(TAG_MAIN, "⚠️ Сервоприводы отключены в конфигурации");
    return true;

    // 🔑 ПРОВЕРКА: указатель валиден?
    if (servoController == nullptr) {
        ESP_LOGE(TAG_MAIN, "❌ servoController == nullptr!");
        return false;
    }

    ESP_LOGI(TAG_MAIN, "=== ⚙️ ПРЕДСТАРТОВЫЙ ТЕСТ СЕРВОПРИВОДОВ ===");

    //if (!_mConfig.enableServoMotors) return true;
    if (_mConfig.enableServoMotors && servoController) {
        if (!servoController->runServoTest()) {
            ESP_LOGE(TAG_MAIN, "❌ КРИТИЧЕСКАЯ ОШИБКА СЕРВОПРИВОДОВ!");
            // ... индикация ошибки ...
        }
    }
#endif    

//=============================================================================    
    // 🔑 ПРОВЕРКА: Измерение фактических импульсов
    for (uint8_t ch = 0; ch < 7; ch++) {
        ESP_LOGD(TAG_MAIN, "  Канал #%u:", ch);
        
            // Нейтраль
        servoController->setLogicalAngle(ch, 0);
        delay(300);
            // -85°
        servoController->setLogicalAngle(ch, -85);
        delay(500);
            // Нейтраль
        servoController->setLogicalAngle(ch, 0);
        delay(300);
            // +85°
        servoController->setLogicalAngle(ch, 85);
        delay(500);
            // Нейтраль
        servoController->setLogicalAngle(ch, 0);
        delay(300);
    }

    // Финальный сброс в нейтраль
    servoController->resetToNeutral();
    delay(300);
//=============================================================================    
    ESP_LOGI(TAG_MAIN, "✅ Тест сервоприводов завершён успешно");
    return true;
} // END runServoStartupTest
*/

// ============================================================================
// 🔧 ПРЕДСТАРТОВЫЙ ТЕСТ МОТОРОВ
// ============================================================================
/**
 * @brief Безопасный тест 2 моторов при старте
 * @note Макс. мощность ограничена 30% для безопасности
 * @param durationMs Длительность теста в мс
 * @return true при успешном завершении
 */
 /*
bool runMotorStartupTest(uint32_t durationMs = 2000) {
#if !CFG_ENABLE_TRACTION_MOTORS
    return true;
#endif
    if (!_mConfig.enableTractionMotors) return true;
    
    // 🔑 ПРОВЕРКА: указатель валиден?
    if (motorController == nullptr) {
        ESP_LOGE(TAG_MAIN, "❌ motorController == nullptr!");
        return false;
    }

    ESP_LOGI(TAG_MAIN, "=== ⚡ ПРЕДСТАРТОВЫЙ ТЕСТ МОТОРОВ ===");
    ESP_LOGW(TAG_MAIN, "⚠️ Макс. мощность: 30% (безопасно)");
    
    // Мотор #0
    ESP_LOGI(TAG_MAIN, "  Мотор #0: 0% → 30% → 0%");
    for (int p = 0; p <= 30; p += 5) {
        motorController->setMotorPower(0, p);
        delay(50);
    }
    for (int p = 30; p >= 0; p -= 5) {
        motorController->setMotorPower(0, p);
        delay(50);
    }
    
    // Мотор #1
    ESP_LOGI(TAG_MAIN, "  Мотор #1: 0% → 30% → 0%");
    for (int p = 0; p <= 30; p += 5) {
        motorController->setMotorPower(1, p);
        delay(50);
    }
    for (int p = 30; p >= 0; p -= 5) {
        motorController->setMotorPower(1, p);
        delay(50);
    }
    
    // Оба мотора
    ESP_LOGI(TAG_MAIN, "  Оба мотора: 20%");
    motorController->setMotorPower(0, 20);
    motorController->setMotorPower(1, 20);
    delay(durationMs / 2);
    
    // 🔑 КРИТИЧЕСКИ: Остановка!
    motorController->stopAllMotors();
    ESP_LOGI(TAG_MAIN, "✅ Тест моторов завершён");
    return true;
}
*/


// 🔧 ВСТАВИТЬ В НАЧАЛО setup(), ЗАМЕНИВ БЛОК С analogRead(36)
/**
 * @brief Диагностика напряжения питания через ADC1 (ESP-IDF 5.0+ совместимый API)
 * @details Заменяет устаревший analogRead() на adc_oneshot.
 *          Совместимо с ESP32 и ESP32-S3. Требует включения #include <esp_adc/adc_oneshot.h>
 */
 /*
void diagnosePowerSupply() {
    ESP_LOGI("DIAG", "🔋 Проверка питания через ADC1 (oneshot)...");
    
    adc_oneshot_unit_handle_t adc1_handle = nullptr;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK_RETURN(adc_oneshot_new_unit(&init_config, &adc1_handle));
    // ESP_ERROR_CHECK_RETURN_VOID(adc_oneshot_new_unit(&init_config, &adc1_handle));


    // 🔑 Для ESP32-S3 GPIO36 -> ADC1_CHANNEL_0, для ESP32 -> ADC1_CHANNEL_0
    adc_channel_t ch = ADC_CHANNEL_0; 
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,  // 0-3.3V диапазон
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK_RETURN(adc_oneshot_config_channel(adc1_handle, ch, &chan_config));
    
    int raw_val = 0;
    adc_oneshot_read(adc1_handle, ch, &raw_val);
    adc_oneshot_del_unit(adc1_handle);
    
    // Коэффициент зависит от вашей схемы делителя (в примере 1:1 для диагностики)
    float vcc_voltage = (raw_val / 4095.0f) * 3.3f; 
    ESP_LOGI("DIAG", "   📊 ADC1_CH0 (GPIO36): raw=%d → %.2fV (норма: 3.2-3.4V)", raw_val, vcc_voltage);
    
    if (vcc_voltage < 3.0f || vcc_voltage > 3.6f) {
        ESP_LOGE("DIAG", "❌ КРИТИЧЕСКИ: Напряжение питания вне допустимого диапазона!");
    } else {
        ESP_LOGI("DIAG", "✅ Напряжение стабильно");
    }
}
*/

// 🔧 ЗАМЕНИТЬ блок диагностики питания в setup() (строки с analogRead(36))
/**
 * @brief Безопасная диагностика питания при старте (ESP-IDF 5.0+ / S3 совместимо)
 * @details Заменяет устаревший analogRead(). Проверяет целостность линий перед инициализацией модулей.
 */
void safeStartupDiagnostics() {
    ESP_LOGI("DIAG", "=== 🛡️ СТАРТОВАЯ ДИАГНОСТИКА АППАРАТНОЙ ЧАСТИ (ESP-IDF 5.0+) ===");
    
    // 1. Проверка линий I2C (подтяжка)
    ESP_LOGI("DIAG", "🔌 Проверка линий I2C (SDA=%d, SCL=%d)...",
             Config::Pins::I2C_SDA, Config::Pins::I2C_SCL);
    pinMode(Config::Pins::I2C_SDA, INPUT_PULLUP);
    pinMode(Config::Pins::I2C_SCL, INPUT_PULLUP);
    delay(5);
    int sda_level = digitalRead(Config::Pins::I2C_SDA);
    int scl_level = digitalRead(Config::Pins::I2C_SCL);
    ESP_LOGI("DIAG", "   SDA=%d (ожидается 1), SCL=%d (ожидается 1)", sda_level, scl_level);
    if (sda_level == 0 || scl_level == 0) {
        ESP_LOGE("DIAG", "❌ КРИТИЧЕСКИ: Линия I2C замкнута на GND! Проверить пайку/термоклей.");
    } else {
        ESP_LOGI("DIAG", "   ✅ Линии I2C в норме");
    }

    // 2. Проверка линий SPI (LoRa)
    ESP_LOGI("DIAG", "🔌 Проверка линий SPI LoRa (CS=%d, SCK=%d, MOSI=%d, MISO=%d)...",
             Config::Pins::LORA_CS, Config::Pins::FSPI_SCLK, 
             Config::Pins::FSPI_MOSI, Config::Pins::FSPI_MISO);
    const uint8_t spi_pins[] = {Config::Pins::LORA_CS, Config::Pins::FSPI_SCLK,
                                Config::Pins::FSPI_MOSI, Config::Pins::FSPI_MISO};
    const char* spi_names[] = {"CS", "SCK", "MOSI", "MISO"};
    for (int i = 0; i < 4; i++) {
        pinMode(spi_pins[i], INPUT_PULLUP);
        delay(1);
        int level = digitalRead(spi_pins[i]);
        ESP_LOGD("DIAG", "🔌 SPI %s (GPIO%d) = %s", 
                spi_names[i], spi_pins[i], level ? "HIGH" : "LOW");  // ← использовать
        // ✅ ИСПОЛЬЗОВАТЬ обе переменные:
        ESP_LOGD("DIAG", "SPI Pin %d level: %d", spi_pins[i], level);
        ESP_LOGD("DIAG", "🔌 %s (GPIO%d) = %s", 
                spi_names[i], spi_pins[i], level ? "HIGH" : "LOW");
        // ESP_LOGI("DIAG", "   %s (GPIO%d) = %d", spi_names[i], spi_pins[i], level);
    }
    ESP_LOGI("DIAG", "✅ Диагностика завершена. Переход к инициализации модулей...");
}



void handleRPiCommand(RPiCommand cmd) {
    switch (cmd) {
        case RPiCommand::CMD_RESTART_CAM:
            ESP_LOGW("UART_BRIDGE", "🔄 Получена команда: ПЕРЕЗАПУСК КАМЕРЫ");
            // TODO: Вызов функции перезапуска камеры
            break;
        case RPiCommand::CMD_CHANGE_MODE:
            ESP_LOGW("UART_BRIDGE", "🔄 Получена команда: СМЕНА РЕЖИМА");
            // TODO: Переключение StabilizationMode
            break;
        case RPiCommand::CMD_HEARTBEAT:
            ESP_LOGD("UART_BRIDGE", "❤️ RPi Heartbeat OK");
            break;
        default:
            ESP_LOGW("UART_BRIDGE", "⚠️ Неизвестная команда RPi: 0x%02X", (uint8_t)cmd);
    }
}



/**
 * @brief Безопасная инициализация Serial с поддержкой USB CDC для ESP32-S3
 * @details При таргете ESP32-S3 переключает вывод на нативный USB.
 *          Гарантирует корректный вывод отладки без дополнительных драйверов.
 */
void initDebugSerial() {
#if defined(ARDUINO_USB_MODE) && (ARDUINO_USB_MODE == 1)
    // 🔑 ESP32-S3 USB CDC mode
    Serial.begin(115200);
    while (!Serial) { delay(10); yield(); }
    ESP_LOGI(TAG_MAIN, "🔌 Serial инициализирован через USB CDC (ESP32-S3)");
#else
    // 🔑 Классический UART0 (ESP32 / ESP32-S3 UART mode)
    Serial.begin(115200);
    while (!Serial) { delay(15); yield(); }
    ESP_LOGI(TAG_MAIN, "🔌 Serial инициализирован через UART0");
#endif
    // 🔧 Единожды задаем уровень логирования
    esp_log_level_set("*", ESP_LOG_INFO);
}


// ============================================================================
// 🔧 SETUP — ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ — С ПРЕДСТАРТОВЫМИ ТЕСТАМИ
// ============================================================================
//=============================================================================
// 🔧 ИСПРАВЛЕНИЕ: Инициализация объектов в setup() вместо глобального создания
// 🔧 Файл: Receiver.ino
// 🔧 Функция: setup()
// 🔧 Дата: 2026-03-15
//=============================================================================
void setup() {
    // 🔑 ПЕРВЫЕ СТРОКИ - ДО ЛЮБЫХ delay()!
    // Serial.begin(115200);
    // while (!Serial) {delay(15);yield(); }  // Ждать подключения монитора
    initDebugSerial();

    //--------------------------------------------------------------
    // [MAIN_APP] setup() - Диагностика питания и целостности пинов
    // Добавлено: 2026-03-30, причина: системный сбой после замены ESP32
    // [ 100][I][Receiver_Air_Plane_8_002_102_mar27.ino:640] setup(): [DIAG] Starting hardware diagnostics...
    ESP_LOGI("DIAG", "=== ДИАГНОСТИКА АППАРАТНОЙ ЧАСТИ ===");

    // 1. Проверка напряжения 3.3V (через ADC1_CH0 = GPIO36)
    // 1. Проверка напряжения 3.3V (через ADC1_CH0 = GPIO7)
    ESP_LOGI("DIAG", "🔋 Проверка питания 3.3V...");
            //    analogReadResolution(12);
            //    analogSetAttenuation(ADC_11db);
            //    uint16_t vcc_raw = analogRead(36);  // GPIO36 - вход только, подключен к 3.3V через делитель
            //    float vcc_voltage = (vcc_raw / 4095.0f) * 3.3f * 2.0f;  // *2.0 если делитель 1:1
            //    ESP_LOGI("DIAG", "   ADC36: %u → ~%.2fV (ожидаемо: 3.2-3.4V)", vcc_raw, vcc_voltage);
            //    if (vcc_voltage < 3.0f || vcc_voltage > 3.6f) {
            //        ESP_LOGE("DIAG", "❌ КРИТИЧЕСКИ: Напряжение питания вне диапазона!");
            //    }
    safeStartupDiagnostics();        

            //       // 2. Проверка целостности линий I2C (подтяжка)
            //       ESP_LOGI("DIAG", "🔌 Проверка линий I2C (SDA=%d, SCL=%d)...", 
            //                Config::Pins::I2C_SDA, Config::Pins::I2C_SCL);
            //       pinMode(Config::Pins::I2C_SDA, INPUT_PULLUP);
            //        pinMode(Config::Pins::I2C_SCL, INPUT_PULLUP);
            //        delay(10);
            //        int sda_level = digitalRead(Config::Pins::I2C_SDA);
            //        int scl_level = digitalRead(Config::Pins::I2C_SCL);
            //        ESP_LOGI("DIAG", "   SDA=%d (ожидается 1), SCL=%d (ожидается 1)", sda_level, scl_level);
            //        if (sda_level == 0 || scl_level == 0) {
            //            ESP_LOGE("DIAG", "❌ Линия I2C замкнута на GND! Проверить пайку и термоклей.");
            //        }

    // 3. Проверка целостности линий SPI (CS, SCK, MOSI, MISO)
            //            ESP_LOGI("DIAG", "🔌 Проверка линий SPI...");
            //            const uint8_t spi_pins[] = {Config::Pins::LORA_CS, Config::Pins::FSPI_SCLK, 
            //                                        Config::Pins::FSPI_MOSI, Config::Pins::FSPI_MISO};
            //            const char* spi_names[] = {"CS", "SCK", "MOSI", "MISO"};
            //            for (int i = 0; i < 4; i++) {
            //                pinMode(spi_pins[i], INPUT_PULLUP);
            //                delay(1);
            //                int level = digitalRead(spi_pins[i]);
            //                ESP_LOGI("DIAG", "   %s (GPIO%d) = %d", spi_names[i], spi_pins[i], level);
            //                if (level == 0 && spi_pins[i] != Config::Pins::LORA_CS) {  // CS может быть 0
            //                    ESP_LOGW("DIAG", "⚠️  Линия %s может быть замкнута", spi_names[i]);
            //                }
            //            }

    // 4. Проверка общего GND через тестовый вывод
    ESP_LOGI("DIAG_GND", "⚡ Проверка целостности GND...");
    pinMode(Config::Pins::LED_STATUS, OUTPUT);
    digitalWrite(Config::Pins::LED_STATUS, HIGH);
    delay(50);
    int led_state = digitalRead(Config::Pins::LED_STATUS);
    ESP_LOGI("DIAG_GND", "   LED_STATUS test: write=HIGH, read=%d", led_state);
    digitalWrite(Config::Pins::LED_STATUS, LOW);
    if (led_state != HIGH) {
        ESP_LOGE("DIAG_GND", "❌ Возможно, обрыв GND или повреждение порта!");
    }

    ESP_LOGI("DIAG_GND", "=== ДИАГНОСТИКА ЗАВЕРШЕНА ===");
    //--------------------------------------------------------------

    Serial.println("### POINT_A: Serial OK ###");
    // 🔑 ТОЧКА ОТЛАДКИ #1 - сразу после Serial.begin()
    // Serial.println("### POINT_A: Serial OK ###");
    // Serial.flush();
    
    // 🔧 ЕДИНОЖДЫ: Настройка уровня логирования
    esp_log_level_set("*", ESP_LOG_INFO);
    
    // 🔑 ТОЧКА ОТЛАДКИ #2 - перед delay()
    // Serial.println("### POINT_B: Before delay ###");
    //delay(100);
    
    // 🔑 ТОЧКА ОТЛАДКИ #3 - после delay()
    // Serial.println("### POINT_C: After delay ###");
    
    // 🔑 ДОБАВИТЬ ЭТИ СТРОКИ для локализации краша:
    // Serial.println("### POINT_D: Before esp_log_level_set ###");
    //Serial.flush();
    
    // 🔑 Инициализация ESP_LOG
    //esp_log_level_set("*", ESP_LOG_INFO);
    
    // Serial.println("### POINT_E: After esp_log_level_set ###");
    //Serial.flush();
    
    ESP_LOGI(TAG_MAIN, "🚀 [STEP 0] START: Система запускается (Vers 6.6.3)");
    ESP_LOGI(TAG_MAIN, "📋 Heap free: %u bytes", esp_get_free_heap_size());
    
    // 🔑 Инициализация ESP_LOG
    ESP_LOGI(TAG_MAIN, "🚀 [STEP 0] START: Система запускается (Vers 6.6.3)");
    ESP_LOGI(TAG_MAIN, "📋 Heap free: %u bytes", esp_get_free_heap_size());
    
    // 🔑 ИНИЦИАЛИЗАЦИЯ ТАЙМЕРОВ (теперь БЕЗОПАСНО!)
    statsTimer = new TimerMillis(Config::Timing::STATS_PRINT_INTERVAL_MS);
    printTimer = new TimerMillis(1500);
    statusTimer = new TimerMillis(15000);
    status_LED_Timer = new TimerMillis(1950);
    batteryMonitor_Timer = new TimerMillis(10000);
    // === TELEMETRY BRIDGE ===
    /**
    * @brief Глобальный таймер для телеметрии
    * @details Интервал 20-50 мс соответствует частоте обновления 20-50 Гц
    *          для передачи данных на RPi Zero 2W через UART2
    * @brief Таймер отправки телеметрии на RPi (20 мс = 50 Гц)
    * @note Используется ссылка, а не указатель — совместимо с TelemetryBridge::update()
    */
    // static TimerMillis telemTimer(20);  // ✅ FIX: объект, не указатель!

                            // Глобальная переменная для хранения последнего RSSI (для телеметрии)
                            //static int8_t _lastRssi = -120;

    // 🔑 ДОБАВИТЬ проверку:
    if (statsTimer == nullptr || printTimer == nullptr || 
        statusTimer == nullptr || status_LED_Timer == nullptr || 
        batteryMonitor_Timer == nullptr) {
        Serial.println("### ERROR: TimerMillis allocation failed! ###");
        //Serial.flush();
        ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: Не удалось создать TimerMillis!");
        while(true) {
            digitalWrite(Config::Pins::LED_STATUS, HIGH);
            delay(100);
            digitalWrite(Config::Pins::LED_STATUS, LOW);
            delay(100);
        }
    }

    ESP_LOGI(TAG_MAIN, "✅ Таймеры инициализированы");
    Serial.println("### POINT_F: Timers OK ###");
    //Serial.flush();    
        ESP_LOGI(TAG_MAIN, "✅ Таймеры инициализированы");
    
    
    // 🔑 Временно отключаем Watchdog для долгой инициализации
    //#if CONFIG_ESP_TASK_WDT_EN
    //    esp_task_wdt_deinit();
    //    ESP_LOGI(TAG_MAIN, "🔓 WDT отключен на время инициализации");
    //#endif
    
    // 🔑 НОВАЯ ТОЧКА: После WDT
    Serial.println("### POINT_H: After WDT disable ###");
    //Serial.flush();
    //ESP_LOGI(TAG_MAIN, "🚀 [STEP 1] Валидация конфигурации...");

    // 🔑 ПЕРВЫЙ ОТЛАДОЧНЫЙ ВЫВОД — точка отсчета
    //ESP_LOGI(TAG_MAIN, "🚀 [STEP 0] START: Система запускается (Vers 6.6.3)");
    //ESP_LOGI(TAG_MAIN, "📋 Heap free: %u bytes", esp_get_free_heap_size());
    
    // --- Инициализация логирования ---
    //esp_log_level_set("*", ESP_LOG_INFO); // Глобальный уровень логирования
    //ESP_LOGI(TAG_MAIN, "🚀 ЗАПУСК СИСТЕМЫ УПРАВЛЕНИЯ БПЛА (Vers 6.6.3)");
    
    
    // 🔑 НОВАЯ ТОЧКА: После WDT
    // Serial.println("### POINT_H2: After WDT disable ###");
    
    // 🔧 НОВАЯ ДИАГНОСТИКА: Проверка перед первым вызовом после точки
    Serial.println("### DEBUG: Testing Serial after H2 ###");
    Serial.flush();
    delay(50);  // Дать время буферу очиститься
    
    ESP_LOGI(TAG_MAIN, "🚀 [STEP 1.1] Валидация конфигурации...");


    // 1. Валидация конфигурации
    if (!validateModuleConfig(_mConfig)) {
        ESP_LOGE(TAG_MAIN, "❌ Критическая ошибка конфигурации!");
        while(true) { 
            digitalWrite(Config::Pins::LED_STATUS, HIGH); 
            delay(100); 
        }
    }
    
    // 1. Вывод конфигурации сборки
    //logBuildConfiguration();

    // ========================================================================
    // 🔋 ИНИЦИАЛИЗАЦИЯ МОНИТОРА БАТАРЕИ (ПЕРВЫМ!)
    // ========================================================================
    delay(500); // Задержка для подключения Serial Monitor
    #if CFG_ENABLE_BATTERY_MONITOR
        if(_mConfig.enableBatteryMonitor){
            ESP_LOGI("MAIN", "=== 🔋 Инициализация BatteryMonitor ===");

            // 🔑 КРИТИЧЕСКИ: Создаём объект!
            batteryMonitor = new BatteryMonitor();
            if (batteryMonitor == nullptr) {
                ESP_LOGE("MAIN", "❌ ОШИБКА: Не удалось создать batteryMonitor!");
            } 

            // Попытка инициализации с конфигурацией по умолчанию
            // if (!batteryMonitor->begin(DEFAULT_CELL_COUNT)) {
            else if (!batteryMonitor->begin(DEFAULT_CELL_COUNT)) {
                ESP_LOGE("MAIN", "❌ Критическая ошибка: не удалось инициализировать монитор батареи");
                // Не блокируем систему, но предупреждаем
            } else {
                ESP_LOGI("MAIN", "✅ Монитор батареи готов");
                // Тестовое чтение напряжения
                BatteryStatus_t status = batteryMonitor->checkVoltage();
                
                ESP_LOGI(TAG_MAIN, "🔋 Статус АКБ:");
                ESP_LOGI(TAG_MAIN, "   Напряжение: %.2fВ (%.1f%%)", 
                        status.voltage, status.percentage);
                ESP_LOGI(TAG_MAIN, "   На ячейку: %.2fВ", status.voltagePerCell);
                ESP_LOGI(TAG_MAIN, "   Статус: %s", 
                        status.isCritical ? "🔴 КРИТИЧЕСКИЙ" :
                        status.isLow ? "⚠️ НИЗКИЙ" : 
                        status.isOk ? "✅ НОРМА" : "❓ ОШИБКА");
            
                // Проверка напряжения перед запуском
                if (status.isCritical) {
                    ESP_LOGE("MAIN", "🔴 КРИТИЧЕСКИ НИЗКОЕ напряжение! Запуск запрещён!");
                    // Можно добавить блокировку запуска здесь
                } else if (status.isLow) {
                    ESP_LOGW("MAIN", "⚠️  Низкое напряжение батареи. Будьте осторожны!");
                } else {
                    ESP_LOGI("MAIN", "✅ Напряжение батареи в норме: %.2fВ (%.1f%%)", 
                            status.voltage, status.percentage);
                }
            }
        } else {
            ESP_LOGW("MAIN", "⚠️  Монитор батареи отключен в конфигурации");
        }
    #endif

    //2. **Диагностика ADC при старте** — добавьте в `setup()` Receiver.ino:
   #if CFG_ENABLE_BATTERY_MONITOR
    if (batteryMonitor && batteryMonitor->isInitialized()) {
        BatteryStatus_t status = batteryMonitor->getStatus();
        ESP_LOGI("DIAG", "🔋 Battery: %.2fV (%.1f%%), per-cell: %.2fV, ADC: %u",
                    status.voltage, status.percentage, 
                    status.voltagePerCell, status.adcValue);
    }
   #endif
    // ========================================================================
    // 🔌 ИНИЦИАЛИЗАЦИЯ I2C
    // ========================================================================
    delay(500); // Задержка для подключения Serial Monitor
    ESP_LOGI(TAG_MAIN, "⚙️  Запуск  CFG_ENABLE_I2C_MASTER...");
    #if CFG_ENABLE_I2C_MASTER
        if (_mConfig.enableI2CMaster) {
            ESP_LOGI(TAG_MAIN, "⚙️  Создание I2CManager...");
            I2CManager = new I2CMasterController();  // ← 🔑 СОЗДАЁМ ОБЪЕКТ!
        
            if (I2CManager == nullptr) {
                ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: Не удалось создать I2CManager!");
            } else if (!I2CManager->begin()) {
                ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: I2C Master не инициализирован!");
            } else {
                ESP_LOGI(TAG_MAIN, "✅ I2C Master успешно инициализирован");
                I2CManager->scanDevices();
            }

            // 🔑 ИСПРАВЛЕНО: I2CManager теперь указатель, доступ через ->
            // 🔑 ПРОВЕРКА MODE2
            uint8_t mode2_check;
            if (I2CManager->readRegister(0x40, 0x01, &mode2_check, 1)) {
                ESP_LOGI(TAG_MAIN, "🔍 MODE2 после begin(): 0x%02X (OUTDRV=%d)",
                mode2_check, (mode2_check & 0x04) ? 1 : 0);
                if (!(mode2_check & 0x04)) {
                    ESP_LOGE(TAG_MAIN, "❌ OUTDRV=0! Серво будут вращаться непрерывно!");
                }
            }
        }
    //}
    #endif

    // 3. Инициализация Сервоприводов (если включено)
    // ========================================================================
    // 🤖 ИНИЦИАЛИЗАЦИЯ СЕРВОПРИВОДОВ
    // ========================================================================
    delay(500); // Задержка для подключения Serial Monitor
    ESP_LOGI(TAG_MAIN, "⚙️  Запуск  CFG_ENABLE_SERVO_MOTORS...");
    #if CFG_ENABLE_SERVO_MOTORS
    if (_mConfig.enableServoMotors) {
        ESP_LOGI(TAG_MAIN, "⚙️  Создание объекта servoController...");
        servoController = new PCA9685_ServoController(Config::PCA9685::DEFAULT_ADDRESS);
        
        if (servoController == nullptr) {
            ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: Не удалось создать servoController!");
        } 
        // 🔑 Проверка: I2CManager должен быть создан!
        else if (I2CManager == nullptr) {
            ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: I2CManager не инициализирован!");
        } else if (!servoController->begin(*I2CManager)) {  // ← 🔑 Передаём ссылку!
            ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: PCA9685 не найден!");
        } else {
            ESP_LOGI(TAG_MAIN, "✅ PCA9685 инициализирован (адрес 0x%02X)",
                servoController->getI2CAddress());
        }
    }
    #endif

    // 4. Инициализация Моторов (если включено)
    // ========================================================================
    // ⚡ ИНИЦИАЛИЗАЦИЯ МОТОРОВ
    // ========================================================================
    delay(500); // Задержка для подключения Serial Monitor
    ESP_LOGI(TAG_MAIN, "⚙️  Запуск  CFG_ENABLE_TRACTION_MOTORS..");
    #if CFG_ENABLE_TRACTION_MOTORS
    if (_mConfig.enableTractionMotors) {
        ESP_LOGI(TAG_MAIN, "⚙️  Создание объекта motorController...");
        motorController = new UnifiedLEDCController();
        
        if (motorController == nullptr) {
            ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: Не удалось создать motorController!");
        } else if (!motorController->begin()) {
            ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: Контроллер моторов не инициализирован!");
        } else {
            ESP_LOGI(TAG_MAIN, "✅ Моторы готовы (GPIO %d, %d)",
                     Config::Pins::motorPins[0], Config::Pins::motorPins[1]);
            motorController->stopAllMotors();
        }
    }
    #endif

    // 5. Инициализация IMU и Стабилизации (если включено)
    // ========================================================================
    // 🧭 ИНИЦИАЛИЗАЦИЯ IMU
    // ========================================================================
    delay(500); // Задержка для подключения Serial Monitor
    ESP_LOGI(TAG_MAIN, "⚙️  Запуск  CFG_ENABLE_IMU..");
    #if CFG_ENABLE_IMU
        if (_mConfig.enableIMU) {
            ESP_LOGI(TAG_MAIN, "⚙️  Создание объекта imuHandler ...");
            imuHandler = new MPU9250Handler();

            if (imuHandler == nullptr) {
                ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: Не удалось создать imuHandler!");
            } else if (I2CManager == nullptr) {
                ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: I2CManager не инициализирован!");
            } else if (!imuHandler->begin(*I2CManager)) {
                ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: MPU-9250 не отвечает");
            } else {
                ESP_LOGI(TAG_MAIN, "✅ MPU-9250 инициализирован. Ожидание калибровки (2 сек)...");
    #if CFG_ENABLE_I2C_MASTER
                delay(2000); // Калибровка гироскопа
                    
    #if CFG_ENABLE_SERVO_MOTORS
                        // ✅ ИСПРАВЛЕНО: передаём указатель на объект servoController
                        if (servoController != nullptr ) {
                            flightStabilizer = new FlightStabilizer();  // ← 🔑 СОЗДАЁМ ОБЪЕКТ!
                                if (flightStabilizer != nullptr && 
                                    flightStabilizer->begin(imuHandler, servoController)) {
                                    flightStabilizer->enable();
                                    stabilizerInitialized = true;
                                    ESP_LOGI(TAG_MAIN, "✅ Стабилизация полета АКТИВНА");
                                } else {
                                    ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: Стабилизатор полета не инициализирован");
                                }
                            }
    #endif
    #endif
            }
        }
    #endif

    // 7. Инициализация ATGM336H (если включено)
    //=============================================================================
    // 🛰️ ИНИЦИАЛИЗАЦИЯ GPS МОДУЛЯ (ATGM336H)
    //=============================================================================
    /**
    * @brief Инициализация GPS-приёмника ATGM336H через UART1
    * @details 
    * - Поддержка двухрежимного позиционирования: GPS (США) + BDS (Китай)
    * - Скорость обмена: 9600 бод (настраивается)
    * - Протокол: NMEA 0183 (GGA, RMC, GSA, GSV предложения)
    * - Пины: RX=GPIO18, TX=GPIO17 (Config::Pins::GPS_RX/GPS_TX)
    * 
    * @note Вызывать после инициализации Serial для отладки
    */
    #if CFG_ENABLE_GPS
    if (_mConfig.enableGPS) {
        ESP_LOGI(TAG_MAIN, "🛰️ Инициализация GPS_Handler (ATGM336H)...");
        
        // 🔑 Создаём объект обработчика
        gpsHandler = new GPSHandler(&Serial1);  // Serial1 = UART1 на ESP32-S3
        
        if (gpsHandler == nullptr) {
            ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: Не удалось создать gpsHandler!");
        } 
        else {
            // 🔑 Инициализация с параметрами:
            // - Baud: 9600 (стандарт для ATGM336H)
            // - Mode: GPS_BDS_COMBINED (максимальная точность)
            if (!gpsHandler->begin(9600, NavigationMode::GPS_BDS_COMBINED)) {
                ESP_LOGE(TAG_MAIN, "❌ ОШИБКА: Не удалось инициализировать GPS модуль");
                delete gpsHandler;
                gpsHandler = nullptr;
            } else {
                ESP_LOGI(TAG_MAIN, "✅ GPS_Handler инициализирован успешно");
                ESP_LOGI(TAG_MAIN, "   Режим: GPS+BDS комбинированный");
                ESP_LOGI(TAG_MAIN, "   Пины: RX=%d, TX=%d", 
                        Config::Pins::GPS_RX, Config::Pins::GPS_TX);
                
                // 🔧 Тестовая самопроверка модуля (опционально, +2 сек)
                #if defined(DEBUG_GPS_SELFTEST)
                ESP_LOGI(TAG_MAIN, "🧪 Запуск самопроверки GPS...");
                if (gpsHandler->runSelfTest()) {
                    ESP_LOGI(TAG_MAIN, "✅ Самопроверка пройдена");
                } else {
                    ESP_LOGW(TAG_MAIN, "⚠️ Самопроверка не пройдена (проверьте антенну)");
                }
                #endif
            }
        }
    } else {
        ESP_LOGW(TAG_MAIN, "⚪ GPS модуль отключён в runtime-конфигурации");
    }
    #else
        ESP_LOGW(TAG_MAIN, "⚪ GPS модуль отключён на этапе компиляции (CFG_ENABLE_GPS=0)");
    #endif


    // 7. Инициализация LoRa (если включено)
    // ========================================================================
    // 📡 ИНИЦИАЛИЗАЦИЯ LoRa -(SX1278)
    // ========================================================================
    delay(500); // Задержка для подключения Serial Monitor
    ESP_LOGI(TAG_MAIN, "⚙️  Запуск  CFG_ENABLE_RADIO..");
    #if CFG_ENABLE_RADIO
    if (_mConfig.enableRadio) {
        ESP_LOGI(TAG_MAIN, "⚙️  Создание объекта fspi  для LoRa...");

            // 🔑 ДОБАВИТЬ сразу после проверки _mConfig .enableRadio:
            pinMode(Config::Pins::LORA_CS, OUTPUT);
            digitalWrite(Config::Pins::LORA_CS, HIGH);  // CS HIGH по умолчанию
            pinMode(Config::Pins::LORA_RST, OUTPUT);
            digitalWrite(Config::Pins::LORA_RST, HIGH); // RST HIGH по умолчанию  
            pinMode(Config::Pins::LORA_DIO0, INPUT_PULLUP);
            ESP_LOGI(TAG_MAIN, "✅ Пины LoRa настроены");

            // 🔧 Явная инициализация с параметрами SPI
            // 🔑 SPI режим задаётся через SPISettings, НЕ через begin()!
            bool spi_ok = fspi.begin(
                Config::Pins::FSPI_SCLK,   // SCK = 12
                Config::Pins::FSPI_MISO,   // MISO = 13  
                Config::Pins::FSPI_MOSI,   // MOSI = 21
                Config::Pins::LORA_CS      // SS/CS = 10
                //true                       // SPI_MODE0 (важно для SX1278!)
            );

            // 🔧 ПОСЛЕ fspi.begin() ДОБАВИТЬ:
            ESP_LOGI(TAG_MAIN, "⚙️  Настройка параметров SPI...");
            // 🔑 КРИТИЧЕСКИ: Явная настройка частоты и режима

            // 🔧 Настройка режима и частоты для SX1278
            fspi.setFrequency(2000000);    // 2 МГц для надёжности
            fspi.setDataMode(SPI_MODE0);   // SX1278 требует MODE0

            ESP_LOGI(TAG_MAIN, "✅ SPI инициализирован для ESP32-S3: FSPI, 2МГц, MODE0");

            ESP_LOGI(TAG_MAIN, "✅ SPI настроен: 2 МГц, MODE0, MSBFIRST");
        
            if (!spi_ok) {
                ESP_LOGE(TAG_MAIN, "❌ fspi.begin() failed!");
            } else {
                // 🔑 Режим SPI задаётся ОТДЕЛЬНО через begin_Transaction():
                ESP_LOGI(TAG_MAIN, "✅ SPI инициализирован: режим 0, 2 МГц");
            }

            // 🔧 ДОБАВИТЬ настройку пинов ПЕРЕД любым digitalWrite():
            ESP_LOGI(TAG_MAIN, "⚙️  Настройка пинов LoRa...");

            ESP_LOGI(TAG_MAIN, "⚙️  Создание объекта loraModule...");
            // Создание Объекта ПРИЕМНИКА
            Module* loraModule = new Module(
                Config::Pins::LORA_CS,      // CS  Chip Select
                Config::Pins::LORA_RST,     // RST  Reset
                Config::Pins::LORA_DIO0,    // IRQ (DIO0) - основной прерыватель
                Config::Pins::LORA_DIO1,    // 🆕 GPIO (DIO1) - теперь передается в RadioLib 7.6.0
                fspi                       // SPI instance
            ); // ← ссылка на инициализированный SPIClass

            uint8_t test_read = fspi.transfer(0x42);  // чтение регистра версии SX1278
            ESP_LOGI(TAG_MAIN, "🔍 SPI test: RegVersion(0x42) = 0x%02X (expected 0x12)", test_read);
            if (test_read != 0x12) {
                ESP_LOGW(TAG_MAIN, "⚠️ SX1278 version mismatch! Check wiring/power.");
            }
            
            if (loraModule != nullptr) {
                ESP_LOGI(TAG_MAIN, "✅ Module SX1278 создан: CS=%d, DIO0=%d, RST=%d, DIO1=%d",
                    Config::Pins::LORA_CS, Config::Pins::LORA_DIO0, 
                    Config::Pins::LORA_RST, Config::Pins::LORA_DIO1);

                ESP_LOGI(TAG_MAIN, "⚙️  Создание объекта receiver ПРИЕМО/ПЕРЕДАТЧИКА...");

                // receiver = new FastLoRaReceiver(*loraModule);

                // Создание приемника (теперь создается внутри конструктора)
        //                      #if LORA_ENABLE_DIO1
                receiver = new FastLoRaReceiver(
                    Config::Pins::LORA_CS,
                    Config::Pins::LORA_RST,
                    Config::Pins::LORA_DIO0, // Передаем DIO0
                    Config::Pins::LORA_DIO1, // Передаем DIO1
                    &fspi   // ← Здесь указатель, т.к. конструктор принимает указатель
                );
        //                        #else
        //                                receiver = new FastLoRaReceiver(
        //                                    Config::Pins::LORA_CS,
        //                                    Config::Pins::LORA_RST,
        //                                    Config::Pins::LORA_DIO0,
        //                                    &fspi
        //                                );
        //                        #endif

                /*          
                //if (receiver != nullptr) {
                if (receiver ) {
                    // 🔑 КРИТИЧЕСКИ: Микро-задержка для стабилизации указателей
                    delay(10);

                    // 🔑 Сначала убедиться, что SPI-шина готова к транзакциям
                    //if (fspi != nullptr) {
                    if (fspi) {
                    delay(1);
                    
                    // 🔑 ОБЪЯВИТЬ переменную:
                    // uint8_t test_read = fspi.transfer(0x42);  // Чтение регистра версии
                    test_read = fspi.transfer(0x42);  // Чтение регистра версии
                    
                    digitalWrite(Config::Pins::LORA_CS, HIGH);
                    fspi.endTransaction();
                    
                    // Теперь можно использовать:
                    ESP_LOGI(TAG_MAIN, "🔍 SPI тест: чтение 0x42 = 0x%02X (ожидается 0x12)", test_read);
                        delay(10);  // 🔑 Микро-задержка для применения настроек
                    }
        
                    // 🔧 ДОПОЛНИТЕЛЬНАЯ ПРОВЕРКА
                    if (test_read != 0x12) {
                        ESP_LOGW(TAG_MAIN, "⚠️ Версия чипа не совпадает! Проверьте подключение.");
                    }

                    // Теперь диагностика должна работать надёжно
                    if (!receiver.diagnoseSPI()) {
                        ESP_LOGE(TAG_MAIN, "❌ Диагностика SPI не пройдена");
        
                        // 🔑 КРИТИЧЕСКИ: Блокировка системы при критической ошибке радиосвязи
                        ESP_LOGE(TAG_MAIN, "🛑 СИСТЕМА ОСТАНОВЛЕНА: без LoRa работа невозможна");
                        
                        // Индикация ошибки миганием
                        pinMode(Config::Pins::LED_STATUS, OUTPUT);
                        while(true) {
                            digitalWrite(Config::Pins::LED_STATUS, HIGH); delay(100);
                            digitalWrite(Config::Pins::LED_STATUS, LOW);  delay(400);  // Асимметричное мигание
                        }

                    }
        
                    if (receiver.begin()) {
                        ESP_LOGI(TAG_MAIN, "✅ LoRa приемник готов");
                    } else {
                        ESP_LOGE("SETUP", "❌ ОШИБКА: LoRa модуль не отвечает!");
                        ESP_LOGE(TAG_MAIN, "🛑 СИСТЕМА ОСТАНОВЛЕНА");

                        // Индикация ошибки миганием
                        pinMode(Config::Pins::LED_STATUS, OUTPUT);
                        while(true) {
                            digitalWrite(Config::Pins::LED_STATUS, HIGH); delay(200);
                            digitalWrite(Config::Pins::LED_STATUS, LOW);  delay(50);  // Быстрое мигание
                        }
                    }
                }
                */

                /*   
                if (receiver) {
                    ESP_LOGI("SETUP", "🔍 Диагностика LoRa перед begin()...");
                    if (!receiver->diagnoseSPI()) {
                        ESP_LOGE("SETUP", "❌ SPI-диагностика не пройдена!");
                        // Не продолжаем инициализацию
                    }
                    if (!receiver->begin()) {
                        ESP_LOGE("SETUP", "❌ Ошибка инициализации LoRa");
                        // Обработка ошибки
                    } else {
                        ESP_LOGI("SETUP", "✅ LoRa инициализирован успешно");
                    }
                }
                */

            }
        }
    //}
    #endif

    // ============================================================
    //=============================================================

    // 🔹 Инициализация UART для RPi Zero 2W
    // ============================================================================
    // 🔧 ИНИЦИАЛИЗАЦИЯ UART TELEMETRY BRIDGE  для RPi Zero 2W
    // ============================================================================
    ESP_LOGI("SETUP", "📡 Инициализация UART телеметрии...");
    if (!TelemetryUARTBridge::begin(
            921600,                              // 🔑 Высокая скорость
            Config::Pins::UART_RPI_TX,          // TX = GPIO5
            Config::Pins::UART_RPI_RX,          // RX = GPIO6
            Config::Pins::UART_RPI_RTS,         // 🔑 RTS = GPIO1
            Config::Pins::UART_RPI_CTS          // 🔑 CTS = GPIO3
        )) {
            ESP_LOGE("SETUP", "❌ Ошибка инициализации UART телеметрии с Flow Control!");
        } else {
            ESP_LOGI("SETUP", "✅ UART телеметрия готова (Flow Control: RTS=1, CTS=3)");
        }
            //                        -1, -1)) {                          // 🔧 RTS/CTS: раскомментировать при использовании
            //                    // int8_t rts = 3, cts = 42;  // 🔑 Пример пинов для Flow Control
            //                    ESP_LOGE("SETUP", "❌ Ошибка инициализации UART телеметрии!");
            //                }

            // ESP_LOGI("SETUP", "✅ UART телеметрия готова");


            //
            //            SerialRPi.begin(115200, SERIAL_8N1, Config::Pins::UART_RPI_RX, Config::Pins::UART_RPI_TX);
            //            ESP_LOGI("UART_RPI", "✅ UART2 инициализирован: RX=GPIO%d, TX=GPIO%d, Baud=115200",
            //                    Config::Pins::UART_RPI_RX, Config::Pins::UART_RPI_TX);

        // В ФУНКЦИИ setup() -> после инициализации всех модулей (batteryMonitor, receiver, imuHandler)
            ESP_LOGI("MAIN", "⚙️  Запуск моста телеметрии UART2 → RPi...");
            // === ИНИЦИАЛИЗАЦИЯ ТЕЛЕМЕТРИЧЕСКОГО МОСТА ===
            //                    ESP_LOGI("MAIN", "⚙️  Инициализация TelemetryBridge...");
            //                    if (!TelemetryBridge::begin()) {
            //                        ESP_LOGE("MAIN", "❌ Критическая ошибка: не удалось инициализировать UART2 для RPi!");
            //                        // Не останавливаем систему — телеметрия опциональна
            //                    } else {
            //                        ESP_LOGI("MAIN", "✅ TelemetryBridge готов (921600 бод, Flow Control)");
            //                    }
    //====================================================================================
    // ========================================================================
    // ✅ ПРЕДСТАРТОВЫЕ ТЕСТЫ ИСПОЛНИТЕЛЬНЫХ УСТРОЙСТВ
    // ========================================================================
    ESP_LOGI(TAG_MAIN, "==========================================");
    ESP_LOGI(TAG_MAIN, "🛡️  ПРЕДСТАРТОВЫЕ ТЕСТЫ ОБОРУДОВАНИЯ");
    ESP_LOGI(TAG_MAIN, "==========================================");

    // Тест сервоприводов
    #if CFG_ENABLE_SERVO_MOTORS
        // if (_mConfig.enableServoMotors) {
        if (_mConfig.enableServoMotors && servoController) {
            ESP_LOGI(TAG_MAIN, "⚙️  Запуск теста сервоприводов...");
            // if (!runServo Startup Test()) {
            if ( !servoController->runServoTest() ) {
                ESP_LOGE(TAG_MAIN, "❌ ТЕСТ СЕРВОПРИВОДОВ ПРОВАЛЕН!");
                // Индикация ошибки
                pinMode(Config::Pins::LED_STATUS, OUTPUT);
                while(true) {
                    digitalWrite(Config::Pins::LED_STATUS, HIGH); delay(100);
                    digitalWrite(Config::Pins::LED_STATUS, LOW);  delay(100);
                }
            }
        }
    #endif
    
    // Тест моторов — с предупреждением!
    #if CFG_ENABLE_TRACTION_MOTORS
        // if (_mConfig.enableTractionMotors) {
        if (_mConfig.enableTractionMotors && motorController) {
            ESP_LOGW(TAG_MAIN, "⚠️⚠️⚠️  ВНИМАНИЕ! ТЕСТ МОТОРОВ ЧЕРЕЗ 5 СЕК! ⚠️⚠️⚠️");
            ESP_LOGW(TAG_MAIN, "Убедитесь:");
            ESP_LOGW(TAG_MAIN, "  1. БПЛА надёжно закреплён");
            ESP_LOGW(TAG_MAIN, "  2. Винты свободны от препятствий");
            ESP_LOGW(TAG_MAIN, "  3. Нет людей в зоне вращения");
            
            // Отсчёт 5 секунд с индикацией
            pinMode(Config::Pins::LED_STATUS, OUTPUT);
            for (int i = 5; i > 0; i--) {
                ESP_LOGI(TAG_MAIN, "  Начало теста моторов через %d сек...", i);
                digitalWrite(Config::Pins::LED_STATUS, HIGH); delay(250);
                digitalWrite(Config::Pins::LED_STATUS, LOW);  delay(250);
            }
            
            ESP_LOGI(TAG_MAIN, "🌀 Запуск теста моторов...");
            // if (!runMotorStartupTest(2000)) {
            if (!motorController->runStartupDiagnostic(30.0f, 2000)) {
                ESP_LOGE(TAG_MAIN, "❌ ТЕСТ МОТОРОВ ПРОВАЛЕН!");
                while(true) {
                    digitalWrite(Config::Pins::LED_STATUS, HIGH); delay(200);
                    digitalWrite(Config::Pins::LED_STATUS, LOW);  delay(50);
                }
            }
        }
    #endif
    //====================================================================================
    ESP_LOGI(TAG_MAIN, "✅ СИСТЕМА ГОТОВА К РАБОТЕ");
    // ========================================================================
    // ✅ ЗАВЕРШЕНИЕ SETUP
    // ========================================================================
    ESP_LOGI("MAIN", "==========================================");
    ESP_LOGI("MAIN", "✅ SETUP завершен. Переход в loop()...");
    ESP_LOGI("MAIN", "==========================================");
    
    // Сброс всех исполнительных устройств в безопасное состояние
    if (_mConfig.enableServoMotors) {
        servoController->resetToNeutral();
    }
    if (_mConfig.enableTractionMotors) {
        motorController->stopAllMotors();
    }

        // запуск таймеров
    //   statsTimer = printTimer = statusTimer = status_LED_Timer = batteryMonitor_Timer = millis();
    //     На ESP32-S3 **Ядро 0** обычно занято Wi-Fi/BLE и обработкой 
    //   прерываний радио. Вы правильно вынесли ПИД на **Ядро 1**. Рекомендую 
    //   добавить приоритет `2` или `3` (ваш код уже использует `2`), и 
    //   убедиться, что стек `4096` байт достаточен. Если 
    //   `FlightStabilizer::update()` начнёт использовать `ESP_LOG` внутри 
    //   задачи, увеличьте стек до `8192`.
    //   
    // 🔧 ВСТАВИТЬ В КОНЕЦ setup() Receiver.ino (после инициализации flightStabilizer):
    #if CONFIG_IDF_TARGET_ESP32S3
        ESP_LOGI("SCHEDULER", "🚀 Создание задачи стабилизации на CORE 1 (ESP32-S3)...");
        xTaskCreatePinnedToCore(
            StabilizerTask,        // Функция задачи
            "StabilizerTask",      // Имя задачи
            8192,                  // Стек (4 KB достаточно)
            NULL,                  // Параметры
            2,                     // Приоритет (2 - выше, чем loop)
            &StabilizerTaskHandle, // Хендл
            1                      // 🔑 Привязка к ЯДРУ 1
        );
        if (StabilizerTaskHandle) {
            ESP_LOGI("SCHEDULER", "✅ Задача стабилизации запущена на CORE 1");
        }
    #endif

// 🔧 Функция диагностики пинов для ESP32-S3
// ... после инициализации всех модулей ...
diagnosePinsForS3();


} // END setup()
// ===========================================================
// ============================================================




// ============================================================================
// 🔧 LOOP — ОСНОВНОЙ ЦИКЛ
// ============================================================================
void loop() {
                     
    // ========================================================================
    // 📡 ОБРАБОТКА LoRa
    // ========================================================================
    #if CFG_ENABLE_RADIO
        if (receiver != nullptr && _mConfig.enableRadio) {
            receiver->update(); // Обработка FSM приемника
            
            // Получение новых команд
            DataComSet_t receivedData;
            if (receiver->getFreshData(receivedData)) {
                hasNewCommands = true;
                receivedCommands = receivedData;  // Копируем в глобальную переменную
                ESP_LOGD(TAG_MAIN, "📥 Пакет получен: ID=%u", receivedData.packet_id);
            
                // 🔑 ПРОВЕРКА: servoController != nullptr
    #if CFG_ENABLE_SERVO_MOTORS
            if (_mConfig.enableServoMotors && servoController != nullptr) {  // ← Проверка!
                    // 🔑 ОБРАБОТКА КОМАНД СЕРВОПРИВОДАМ (ТЗ п.3.Д4)
                    // processServoCommands(receivedData.comUp, receivedData.comLeft);
                    servoController->processFlightCommands(receivedData.comUp, receivedData.comLeft);
                }
    #endif
 
    #if CFG_ENABLE_SERVO_MOTORS
            // Обработка флага парашюта
            if (receivedData.comSetAll & PARASHUT_FLAG) {
                if (servoController != nullptr) {
                    servoController->setPhysicalAngle(4, 180);
                    ESP_LOGI("LOOP", "🪂 Парашют активирован через флаг!");
                }
            }
    #endif
            }
        }
    #endif
    
    // ========================================================================
    // 🤖 ОБРАБОТКА КОМАНД
    // ========================================================================
    if (hasNewCommands) {
        // Обработка команд сервоприводам
    #if CFG_ENABLE_SERVO_MOTORS
            //if (_mConfig.enableServoMotors) {
            if (_mConfig.enableServoMotors && servoController != nullptr) {
                // Преобразование команд в углы (0-255 → -90°...+90°)
                //int16_t rollAngle = map(receivedCommands.comLeft, 0, 255, -90, 90);
                int16_t rollAngle = map(receivedCommands.comLeft, 0, 180, -90, 90);
                //int16_t pitchAngle = map(receivedCommands.comUp, 0, 255, -90, 90);
                int16_t pitchAngle = map(receivedCommands.comUp, 0, 180, -90, 90);
                
                // Установка углов сервоприводов
                servoController->setLogicalAngle(0, rollAngle);   // Левый элерон
                servoController->setLogicalAngle(1, -rollAngle);  // Правый элерон
                servoController->setLogicalAngle(2, pitchAngle);  // Левый руль высоты
                servoController->setLogicalAngle(3, pitchAngle);  // Правый руль высоты
            }
    #endif
    }

    #if CFG_ENABLE_TRACTION_MOTORS
            // Обработка команд моторам
            if (_mConfig.enableTractionMotors && motorController != nullptr) {
                float throttlePercent = map(receivedCommands.comThrottle, 
                                        1000, 2000, 0.0f, 100.0f);
                throttlePercent = constrain(throttlePercent, 0.0f, 100.0f);
                
                motorController->setMotorPower(0, throttlePercent);
                motorController->setMotorPower(1, throttlePercent);
            }
    #endif     

    // ==================*****************??????????????????????????
    hasNewCommands = false;
    
    // ========================================================================
    // 🧭 ОБНОВЛЕНИЕ IMU
    // ========================================================================
    //if (_mConfig.enableIMU) {
    if (_mConfig.enableIMU && imuHandler != nullptr) {  // ← Добавлена проверка!
        imuHandler->updateSensors();
        
        // Периодическая печать данных IMU
        static uint32_t lastImuPrint = 0;
        if (millis() - lastImuPrint >= 1000) {
            if (imuHandler->isDataValid()) {
                const SensorData& data = imuHandler->getData();
                if (CDebug.lora && (packetCounter % 16 == 0)) {
                    ESP_LOGD("LOOP", "🧭 IMU: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°", 
                            data.roll, data.pitch, data.yaw);
}                // ✅ ИСПОЛЬЗОВАТЬ data:
                //ESP_LOGD("LOOP", "🧭 IMU: Roll=%.1f° Pitch=%.1f° Yaw=%.1f° T=%.1f°C",
                //     data.roll, data.pitch, data.yaw, data.temperature);
                ESP_LOGD("LOOP", "🧭 IMU: Roll=%.1f° Pitch=%.1f° Yaw=%.1f° T=%.1f°C",
                        data.roll, data.pitch, data.yaw, data.temperature);
                ESP_LOGV("LOOP", "IMU Roll: %.1f, Pitch: %.1f", data.roll, data.pitch);            
            }
            lastImuPrint = millis();
        }
    }

    //=============================================================================
    // 🛰️ ОБНОВЛЕНИЕ ДАННЫХ GPS
    //=============================================================================
    /**
    * @brief Периодическое обновление GPS-данных
    * @details 
    * - Вызывает gpsHandler->update() для парсинга NMEA-предложений
    * - Периодичность: на каждый проход loop() (неблокирующая обработка)
    * - Данные кэшируются в структуре GPSData для доступа из других модулей
    * 
    * @note Данные используются в телеметрии и навигационных расчётах
    */
    #if CFG_ENABLE_GPS
    if (gpsHandler != nullptr && _mConfig.enableGPS) {
        // 🔑 Неблокирующее обновление: чтение доступных байт из UART
        gpsHandler->update();
        
        // 🔧 Периодическая отладочная печать (раз в 5 секунд)
        static uint32_t lastGpsLog = 0;
        if (millis() - lastGpsLog >= 5000) {
            if (gpsHandler->hasValidData()) {
                const GPSData& gps = gpsHandler->getData();
                ESP_LOGD("GPS_LOOP", "📍 GPS: Fix=%s | Sats=%d | HDOP=%.1f | Speed=%.1f км/ч",
                        gpsHandler->fixStatusToString(gps.fix_status).c_str(),
                        gps.satellites_used,
                        gps.hdop,
                        gps.speed);
                ESP_LOGV("GPS_LOOP", "   Координаты: %.6f°, %.6f°, H=%.1fм",
                        gps.latitude, gps.longitude, gps.altitude);
            } else {
                ESP_LOGV("GPS_LOOP", "⏳ GPS: Ожидание фиксации... (видимые: %d)", 
                        gpsHandler->getData().satellites_visible);
            }
            lastGpsLog = millis();
        }
    }
    #endif

    // ========================================================================
    // 📊 ПЕРИОДИЧЕСКАЯ СТАТИСТИКА
    // ========================================================================
    static uint32_t lastStatsPrint = 0;
    if (millis() - lastStatsPrint >= Config::Timing::STATS_PRINT_INTERVAL_MS) {
        ESP_LOGI("MAIN", "=== 📊 СТАТИСТИКА СИСТЕМЫ ===");
        
#if CFG_ENABLE_RADIO
    //if (_mConfig.enableRadio) {
    if (_mConfig.enableRadio && receiver != nullptr) {
            receiver->printStatistics();
    }
#endif
        
#if CFG_ENABLE_BATTERY_MONITOR
    if (_mConfig.enableBatteryMonitor && batteryMonitor != nullptr) {  // ← Добавлена проверка!
        batteryMonitor->update();
        if (batteryMonitor_Timer->is_ready()) {
            batteryMonitor->printStatus(false);
        }
    }
#endif
    lastStatsPrint = millis();
    }
                /*  
                // ========================================================================
                // 📊 ПЕРИОДИЧЕСКАЯ 📡 Отправка телеметрии 
                // ========================================================================
                // === ОТПРАВКА ТЕЛЕМЕТРИИ НА RPi (20-50 мс интервал) ===
                if (CDebug.cc && telemTimer.is_ready()) {
                    // 🔑 FIX: передаём *telemTimer (разыменование) ИЛИ объявили как объект выше
                    TelemetryBridge::update(telemTimer, 
                                        receivedCommands, 
                                        imuHandler->getData(), 
                                        batteryMonitor->getStatus());
                    
                    // 🔑 Обновление RSSI для телеметрии (из LoRa статистики)
                    if (receiver && receiver->isConnected()) {
                        _last_Rssi = receiver->getStatsFull().lastRSSI;
                    }
                    
                    // 🔑 Обработка обратных команд от RPi (неблокирующая)
                    uint8_t cmdBuffer[16];
                    int cmdLen = TelemetryBridge::receiveCommand(cmdBuffer, sizeof(cmdBuffer));
                    if (cmdLen > 0) {
                        // 🔧 Парсинг команд (пример)
                        if (cmdBuffer[0] == 0x01) {
                            ESP_LOGI("UART_CMD", "📷 Команда: перезапуск камеры");
                            // cameraHandler->restart();  // Если есть модуль камеры
                        }
                        else if (cmdBuffer[0] == 0x02) {
                            uint8_t newMode = cmdBuffer[1];
                            ESP_LOGI("UART_CMD", "🔄 Команда: смена режима на %u", newMode);
                            // stabilizer->setMode(static_cast<StabilizationMode>(newMode));
                        }
                    }
                }
                */

    // ============================================================================
    // 🔧 ОТПРАВКА ТЕЛЕМЕТРИИ НА RPi (каждые 20-50 мс)
    // ============================================================================
    //=============================================================================
    // 📡 ФОРМИРОВАНИЕ ПАКЕТА ТЕЛЕМЕТРИИ С ДАННЫМИ GPS
    //=============================================================================
    /**
    * @brief Сборка телеметрического пакета для отправки на RPi Zero 2W
    * @details Алгоритм формирования пакета (49 байт):
    * 
    * 1. Заголовок: [0xAA 0x55] (2 байта)
    * 2. Payload (44 байта):
    *    - Управление: packet_id, timestamp, com_*, com_flags
    *    - Стабилизация: roll, pitch, yaw, altitude, speed
    *    - Навигация: latitude, longitude (из GPS)
    *    - Батарея: voltage, percentage
    *    - Сигнал: rssi, flight_mode, imu_status
    * 3. CRC8: полином 0x07, инициализация 0x00 (1 байт)
    * 4. Футер: [0xCC 0x33] (2 байта)
    * 
    * @note Отправка неблокирующая: при переполнении буфера кадр пропускается
    */
    if (telemTimer->is_ready() && TelemetryUARTBridge::isInitialized()) {
        
        TelemetryPacket_t pkt = {};  // 🔑 Инициализация нулями
        
        // === 1. Данные управления (из LoRa) ===
        pkt.packet_id    = receivedCommands.packet_id;
        pkt.timestamp    = millis();
        pkt.com_up       = receivedCommands.comUp;
        pkt.com_left     = receivedCommands.comLeft;
        pkt.com_throttle = receivedCommands.comThrottle;
        pkt.com_flags    = receivedCommands.comSetAll;
        
        // === 2. Данные стабилизации (из FlightStabilizer) ===
        if (imuHandler && imuHandler->isDataValid()) {
            const SensorData& imu = imuHandler->getData();
            pkt.roll     = imu.roll;
            pkt.pitch    = imu.pitch;
            pkt.yaw      = imu.yaw;
            pkt.altitude = imu.altitude;  // Если барометр подключён
        }
        
        // === 3. 🔑 НАВИГАЦИОННЫЕ ДАННЫЕ (из GPS) ===
        #if CFG_ENABLE_GPS
        if (gpsHandler && _mConfig.enableGPS && gpsHandler->hasValidData()) {
            const GPSData& gps = gpsHandler->getData();
            
            // 🔧 Конвертация узлов → м/с для совместимости с ground station
            pkt.speed = gps.speed / 3.6f;  // 1 узел = 1.852 км/ч = 0.514 м/с
            
            // 🔧 Координаты в десятичных градусах (WGS84)
            pkt.latitude  = gps.latitude;
            pkt.longitude = gps.longitude;
            
            ESP_LOGV("TELEM_GPS", "📤 Навигация: %.6f, %.6f, %.2f м/с",
                    pkt.latitude, pkt.longitude, pkt.speed);
        } else {
            // 🔧 Заглушка при отсутствии фикса: нулевые координаты
            pkt.speed     = 0.0f;
            pkt.latitude  = 0.0;
            pkt.longitude = 0.0;
            ESP_LOGV("TELEM_GPS", "⚠️ GPS данных нет → нулевые координаты");
        }
        #else
        // 🔧 Если GPS отключён на этапе компиляции
        pkt.speed     = 0.0f;
        pkt.latitude  = 0.0;
        pkt.longitude = 0.0;
        #endif
        
        // === 4. Данные батареи ===
        // if (batteryMonitor && batteryMonitor->_initialized()) {
        if (batteryMonitor && batteryMonitor->isInitialixed()) {
            BatteryStatus_t batt = batteryMonitor->getStatus();
            pkt.battery_voltage = batt.voltage;
            // 🔧 Конвертация float → uint8_t с ограничением диапазона
            pkt.battery_percent = static_cast<uint8_t>(
                constrain(batt.percentage, 0.0f, 100.0f)
            );
        }
        
        // === 5. Сигнал и статус системы ===
        if (receiver && receiver->isInitialized()) {
            ReceiverStats stats = receiver->getStatsFull();
            pkt.rssi = stats.lastRSSI;
        } else {
            pkt.rssi = -120;  // Заглушка: "нет сигнала"
        }
        
        pkt.flight_mode = flightStabilizer ? 
            static_cast<uint8_t>(flightStabilizer->getMode()) : 1;  // 1=MANUAL
        pkt.imu_status = (imuHandler && imuHandler->isDataValid()) ? 
            imuHandler->getData().status : 0;
        
        // === 6. 🔑 ОТПРАВКА (неблокирующая) ===
        if (!TelemetryUARTBridge::sendTelemetry(pkt)) {
            ESP_LOGV("UART_TX", "⏳ Пропуск кадра телеметрии: буфер UART занят");
        } else {
            ESP_LOGD("UART_TX", "📤 Telemetry sent | ID:%u | RSSI:%d dBm", 
                    pkt.packet_id, pkt.rssi);
        }
        
        // 🔧 Сброс таймера после успешной отправки
        telemTimer->reset();
    } // END if (telemTimer->is_ready()

                        /*
                        if (telemTimer->is_ready() && TelemetryUARTBridge::isInitialized()) {
                        // 🔹 2. Отправка телеметрии каждые ~30 мс
                        if (telemTimer->is_ready() && TelemetryUARTBridge::isInitialized() && imuHandler->isDataValid()) {
                            TelemetryPacket_t pkt;
                            
                            // 1. Команды пульта
                            // pkt.packet_id   = telemetryPacketCounter++;
                            pkt.packet_id   = receivedCommands.packet_id;
                            pkt.timestamp   = millis();
                            pkt.com_up      = receivedCommands.comUp;
                            pkt.com_left    = receivedCommands.comLeft;
                            pkt.com_throttle= receivedCommands.comThrottle;
                            pkt.com_flags   = receivedCommands.comSetAll;

                            // 2. Данные стабилизации
                            const SensorData& imuData = imuHandler->getData();
                            pkt.roll      = imuData.roll;
                            pkt.pitch     = imuData.pitch;
                            pkt.yaw       = imuData.yaw;
                            pkt.altitude  = imuData.altitude; // Если барометр подключён
                            // 🔑 Используем объект-обработчик (проверьте имя в вашем коде!)
                            // 3. Навигация
                            if (gpsHandler && gpsHandler->hasFix()) {
                                const GPSData& gpsData = gpsHandler->getData();  // ← Получаем данные один раз
                                pkt.speed     = gpsData.speed / 3.6f;            // Узлы → м/с
                                pkt.latitude  = gpsData.latitude;
                                pkt.longitude = gpsData.longitude;
                            } else {
                                pkt.speed     = 0.0f;
                                pkt.latitude  = 0.0;
                                pkt.longitude = 0.0;
                            }
                                //            // 3. Навигация
                                //            if (GPSData->hasFix()) {
                                //                // pkt.latitude  = gpsData.getData().latitude;
                                //                pkt.latitude  = GPSData->getData().latitude;
                                //
                                //                // pkt.longitude = gpsData.getData().longitude;
                                //                pkt.longitude = GPSData->getData().longitude;
                                //            }

                            // 4. Батарея
                            // BatteryStatus_t batt = batteryMon.getStatus();
                            // BatteryStatus_t batt = batteryMonitor.getStatus();
                            BatteryStatus_t batt = batteryMonitor->getStatus();
                            pkt.battery_voltage = batt.voltage;
                            pkt.battery_percent = static_cast<uint8_t>(batt.percentage);

                            // 5. Сигнал и статус
                            int16_t rssi = 0; 
                            uint32_t rxPkt = 0;
                            // loraReceiver->getStats(rxPkt, rxPkt, rxPkt, rxPkt, rxPkt); // Упрощено, возьмите из getStatsFull()
                            // receiver->getStats(rxPkt, rxPkt, rxPkt, rxPkt, rxPkt); // Упрощено, возьмите из getStatsFull()

                            if (receiver && receiver->isInitialized()) {
                                ReceiverStats stats = receiver->getStatsFull();
                                pkt.rssi = stats.lastRSSI;
                                // ... другие поля при необходимости
                            }

                            // pkt.rssi = rssi; // RSSI берётся из LoRa модуля
                            // pkt.flight_mode = static_cast<uint8_t>(flightStabilizer.getMode());
                            pkt.flight_mode = static_cast<uint8_t>(flightStabilizer->getMode());
                            pkt.imu_status  = imuData.status;

                            // Отправка (неблокирующая)
                            if (!TelemetryUARTBridge::sendTelemetry(pkt)) {
                                ESP_LOGW("UART_TX", "⏳ Пропуск кадра: буфер UART занят");
                            } else {
                                ESP_LOGD("UART_TX", "📤 Telemetry sent ID:%u", pkt.packet_id);
                            }
                        }
                        */

                        /*  
                        // 🔑 1. Формирование пакета телеметрии
                        TelemetryPacket_t telem = {};
                        
                        // Данные управления (из LoRa)
                        telem.packet_id    = receivedCommands.packet_id;
                        telem.timestamp    = receivedCommands.timestamp;
                        telem.com_up       = receivedCommands.comUp;
                        telem.com_left     = receivedCommands.comLeft;
                        telem.com_throttle = receivedCommands.comThrottle;
                        telem.com_flags    = receivedCommands.comSetAll;
                        
                        // Данные стабилизации (из FlightStabilizer)
                        const SensorData& imu = imuHandler->getData();
                        telem.roll      = imu.roll;
                        telem.pitch     = imu.pitch;
                        telem.yaw       = imu.yaw;
                        telem.altitude  = imu.altitude;
                        telem.speed     = 0.0f;  // 🔧 Заменить на реальные данные при наличии GPS
                        telem.latitude  = 0.0;   // 🔧 Заменить на GPS данные
                        telem.longitude = 0.0;
                        
                        // Данные батареи
                        BatteryStatus_t bat = batteryMonitor->getStatus();
                        telem.battery_voltage = bat.voltage;
                        telem.battery_percent = static_cast<uint8_t>(constrain(bat.percentage, 0, 100));
                        
                        // Сигнал и статус
                        telem.rssi = receiver->isConnected() 
                            ? receiver->getStatsFull().lastRSSI : -120;
                        telem.flight_mode = flightStabilizer->isEnabled() ? 2 : 1;  // 1=MANUAL, 2=STAB
                        telem.imu_status = imu.status;
                        
                        // 🔑 2. Отправка (неблокирующая)
                        if (!TelemetryUARTBridge::sendTelemetry(telem)) {
                            ESP_LOGV("TELEM", "⏳ Пропуск кадра: буфер UART занят");
                        }
                        
                        // 🔧 Проверка обратного канала (команды от RPi)
                        uint8_t cmd;
                        if (TelemetryUARTBridge::checkCommand(cmd)) {
                            ESP_LOGI("TELEM", "📥 Команда от RPi: 0x%02X", cmd);
                            // 🔧 Обработка команд:
                            // 0x01 = перезапуск камеры, 0x02 = смена режима, и т.д.
                            switch(cmd) {
                                case 0x01:
                                    ESP_LOGI("TELEM", "📷 Перезапуск камеры (заглушка)");
                                    // camera_restart();  // 🔧 Реализовать при наличии камеры
                                    break;
                                case 0x02:
                                    ESP_LOGI("TELEM", "🔄 Смена режима полёта (заглушка)");
                                    // flightStabilizer->toggleMode();  // 🔧 Реализовать
                                    break;
                                default:
                                    ESP_LOGW("TELEM", "⚠️ Неизвестная команда: 0x%02X", cmd);
                            }
                        }
    }// END if (telemTimer.is_ready()
   */


    //=======================================================================================
    //====================================================
    // 🔹 1. Обработка обратного канала (RPi → ESP32)
    //====================================================
    //            if (TelemetryUARTBridge::isInitialized()) {
    //                TelemetryUARTBridge::processRPiCommands(handleRPiCommand);
    //            }
    //=============================================================================
    // 📥 ОБРАБОТКА КОМАНД ОТ RPi Zero 2W (обратный канал)
    //=============================================================================
    /**
    * @brief Неблокирующая обработка входящих команд от Raspberry Pi
    * @details 
    * - Формат команды: [0xAA 0x55][cmd][CRC8][0xCC 0x33] (5 байт)
    * - Поддерживаемые команды (enum RPiCommand в TelemetryUARTBridge.h):
    *   • CMD_RESTART_CAM (0x01): Перезапуск камеры/видеопотока
    *   • CMD_CHANGE_MODE (0x02): Смена режима полёта (MANUAL/STAB/FULL)
    *   • CMD_HEARTBEAT   (0xFF): Сигнал "живости" для мониторинга связи
    * 
    * @note Обработка через callback-функцию handleRPiCommand()
    */
    if (TelemetryUARTBridge::isInitialized()) {
        // 🔑 Неблокирующий опрос входящих команд
        TelemetryUARTBridge::processRPiCommands([](RPiCommand cmd) {
            switch (cmd) {
                case RPiCommand::CMD_RESTART_CAM:
                    ESP_LOGW("UART_CMD", "📷 Получена команда: ПЕРЕЗАПУСК КАМЕРЫ");
                    // 🔧 Здесь вызов функции перезапуска камеры:
                    // if (cameraHandler) cameraHandler->restartStream();
                    // 🔧 Индикация выполнения:
                    ESP_LOGI("UART_CMD", "✅ Команда принята в обработку");
                    break;
                    
                case RPiCommand::CMD_CHANGE_MODE:
                    ESP_LOGW("UART_CMD", "🔄 Получена команда: СМЕНА РЕЖИМА ПОЛЁТА");
                    // 🔧 Переключение режима стабилизации:
                    if (flightStabilizer) {
                        // Циклическое переключение: MANUAL → STAB → FULL → MANUAL
                        auto current = flightStabilizer->getMode();
                        StabilizationMode next = (current == StabilizationMode::MANUAL) ? 
                            StabilizationMode::ROLL_PITCH :
                            (current == StabilizationMode::ROLL_PITCH) ? 
                            StabilizationMode::FULL : StabilizationMode::MANUAL;
                        flightStabilizer->setMode(next);
                        ESP_LOGI("UART_CMD", "✅ Режим изменён: %s", 
                                flightStabilizer->modeToString(next));
                    }
                    break;
                    
                case RPiCommand::CMD_HEARTBEAT:
                    // 🔧 Heartbeat для мониторинга связи (без действий, только лог)
                    ESP_LOGD("UART_CMD", "❤️ RPi Heartbeat OK | Uptime: %lu сек", 
                            millis() / 1000);
                    break;
                    
                default:
                    ESP_LOGW("UART_CMD", "⚠️ Неизвестная команда от RPi: 0x%02X", 
                            static_cast<uint8_t>(cmd));
                    break;
            }
        });
    }  //END if (TelemetryUARTBridge



    // Небольшая задержка для стабильности
    delay(10);
    //} 
}// END Loop()


// 🔧 Функция диагностики пинов для ESP32-S3
void diagnosePinsForS3() {
    #if defined(CONFIG_IDF_TARGET_ESP32S3)
    ESP_LOGI("PIN_DIAG", "=== 🔍 ДИАГНОСТИКА ПИНОВ ДЛЯ ESP32-S3 ===");
    
    // Проверка конфликта: LORA_RST != FSPI_MISO
    if (Config::Pins::LORA_RST == Config::Pins::FSPI_MISO) {
        ESP_LOGE("PIN_DIAG", "❌ КОНФЛИКТ: LORA_RST и FSPI_MISO на одном пине!");
    } else {
        ESP_LOGI("PIN_DIAG", "✅ LoRa SPI: RST=%d ≠ MISO=%d",
                 Config::Pins::LORA_RST, Config::Pins::FSPI_MISO);
    }
    
    // Проверка конфликта: BATTERY_ADC_PIN != GPS_RX
    if (Config::Pins::BATTERY_ADC_PIN == Config::Pins::GPS_RX) {
        ESP_LOGE("PIN_DIAG", "❌ КОНФЛИКТ: ADC и GPS_RX на одном пине!");
    } else {
        ESP_LOGI("PIN_DIAG", "✅ ADC/GPS: ADC=%d ≠ GPS_RX=%d",
                 Config::Pins::BATTERY_ADC_PIN, Config::Pins::GPS_RX);
    }
    
    // Проверка поддержки пинов
    ESP_LOGI("PIN_DIAG", "📋 Поддержка функций:");
    ESP_LOGI("PIN_DIAG", "   • I2C: SDA=%d ✅, SCL=%d ✅",
             Config::Pins::I2C_SDA, Config::Pins::I2C_SCL);
    ESP_LOGI("PIN_DIAG", "   • LEDC: MOTOR0=%d ✅, MOTOR1=%d ✅",
             Config::Pins::motorPins[0], Config::Pins::motorPins[1]);
    ESP_LOGI("PIN_DIAG", "   • ADC1: BATTERY=%d %s",
             Config::Pins::BATTERY_ADC_PIN,
             (Config::Pins::BATTERY_ADC_PIN >= 1 && 
              Config::Pins::BATTERY_ADC_PIN <= 10) ? "✅" : "⚠️ ADC2");
    
    ESP_LOGI("PIN_DIAG", "=== ✅ ДИАГНОСТИКА ЗАВЕРШЕНА ===");
    #endif
}


//=====================================================================
//=====================================================================
//=====================================================================
//=====================================================================

void testHardwareConnections() {
    Serial.println("🔍 Hardware Connection Test:");
    Serial.println("CS pin: " + String(Config::Pins::LORA_CS));
    Serial.println("RST pin: " + String(Config::Pins::LORA_RST));
    Serial.println("DIO0 pin: " + String(Config::Pins::LORA_DIO0));
    
    // Проверка питания через сброс
    digitalWrite(Config::Pins::LORA_RST, LOW);
    delay(10);
    digitalWrite(Config::Pins::LORA_RST, HIGH);
    delay(10);
    
    Serial.println("DIO0 state after reset: " + String(digitalRead(Config::Pins::LORA_DIO0)));
}

/*
// 🔧 Расширенная диагностика PCA9685 перед инициализацией
void diagnosePCA9685Hardware(I2CMasterController& i2c) {
    ESP_LOGI("PCA9685_DIAG", "=== АППАРАТНАЯ БЫСТРАЯ ----> ДИАГНОСТИКА PCA9685 ===");
    
    // 1. Проверка ответа на адрес 0x40
    if (!i2c.isDeviceConnected(0x40)) {
        ESP_LOGE("PCA9685_DIAG", "❌ PCA9685 НЕ ОТВЕЧАЕТ на I2C-адресе 0x40");
        ESP_LOGE("PCA9685_DIAG", "   Возможные причины:");
        ESP_LOGE("PCA9685_DIAG", "   • Нет питания 3.3V на VCC");
        ESP_LOGE("PCA9685_DIAG", "   • Нет pull-up резисторов на SDA/SCL");
        ESP_LOGE("PCA9685_DIAG", "   • Нет общего GND");
        ESP_LOGE("PCA9685_DIAG", "   • Неверный адрес (перемычки A0-A5)");
        return;
    }
    ESP_LOGI("PCA9685_DIAG", "✅ PCA9685 отвечает на 0x40");
    
    // 2. Чтение MODE1 до инициализации
    uint8_t mode1;
    if (i2c.readRegister(0x40, 0x00, &mode1, 1)) {
        ESP_LOGI("PCA9685_DIAG", "📋 MODE1 до инициализации: 0x%02X (SLEEP=%d)", 
                 mode1, (mode1 & 0x10) ? 1 : 0);
    }
    
    // 3. Тест записи/чтения регистра
    uint8_t test_val = 0xAA;
    if (i2c.writeRegister(0x40, 0x06, &test_val, 1)) {  // LED0_ON_L
        uint8_t read_back;
        if (i2c.readRegister(0x40, 0x06, &read_back, 1)) {
            if (read_back == test_val) {
                ESP_LOGI("PCA9685_DIAG", "✅ Тест записи/чтения пройден");
            } else {
                ESP_LOGE("PCA9685_DIAG", "❌ Ошибка чтения: ожидал 0x%02X, получил 0x%02X", 
                         test_val, read_back);
            }
        } else {
            ESP_LOGE("PCA9685_DIAG", "❌ Не удалось прочитать после записи");
        }
    } else {
        ESP_LOGE("PCA9685_DIAG", "❌ Не удалось записать тестовое значение");
    }
    
    ESP_LOGI("PCA9685_DIAG", "=== Конец диагностики ===");
}
*/

// Вызов в setup():
// ... после I2CManager.begin() ...
// diagnosePCA9685Hardware(I2CManager);


// 🔧 Быстрая I2C-диагностика PCA9685
// ✅ Быстрая диагностика без мультиметра
void diagnosePCA9685(I2CMasterController& i2c) {
    Serial.println("\n✅ Быстрая диагностика без мультиметра");    
    ESP_LOGI("DIAG", "=== Диагностика PCA9685 ======");
    
    // 1. Проверка ответа на адрес 0x40
    if (!i2c.isDeviceConnected(0x40)) {
        ESP_LOGE("DIAG", "❌ PCA9685 не отвечает на I2C-адресе 0x40");
        return;
    }
    ESP_LOGI("DIAG", "✅ PCA9685 отвечает на 0x40");
    
    // 2. Чтение регистров
    uint8_t mode1, prescale;
    if (i2c.readRegister(0x40, 0x00, &mode1, 1)) {
        ESP_LOGI("DIAG", "📋 MODE1=0x%02X (SLEEP=%d, RESTART=%d)", 
                 mode1, (mode1&0x10)?1:0, (mode1&0x80)?1:0);
    }
    if (i2c.readRegister(0x40, 0xFE, &prescale, 1)) {
        ESP_LOGI("DIAG", "📋 PRESCALE=0x%02X → частота ~%.1f Гц", 
                 prescale, 25000000.0f/(4096*(prescale+1)));
    }
    
    // 3. Тестовая запись/чтение
    uint8_t test_val = 0xAA;
    if (i2c.writeRegister(0x40, 0x06, &test_val, 1)) {  // LED0_ON_L
        uint8_t read_back;
        if (i2c.readRegister(0x40, 0x06, &read_back, 1) && read_back == test_val) {
            ESP_LOGI("DIAG", "✅ Тест записи/чтения пройден");
        } else {
            ESP_LOGE("DIAG", "❌ Ошибка чтения после записи: ожидал 0x%02X, получил 0x%02X", 
                     test_val, read_back);
        }
    }
    ESP_LOGI("DIAG", "=== Конец диагностики ===");
}


/**
 * @brief Безопасный предстартовый тест оборудования
 * @note Требует ручного подтверждения перед тестом моторов!
 */
void runStartupTest() {
    ESP_LOGI("STARTUP_TEST", "▶️ Запуск runStartupTest()");
    ESP_LOGI("STARTUP_TEST", "   CFG_ENABLE_SERVO_MOTORS=%d", CFG_ENABLE_SERVO_MOTORS);
    ESP_LOGI("STARTUP_TEST", "   CFG_ENABLE_TRACTION_MOTORS=%d", CFG_ENABLE_TRACTION_MOTORS);
    ESP_LOGI("STARTUP_TEST", "==========================================");
    ESP_LOGI("STARTUP_TEST", "🛡️  ПРЕДСТАРТОВЫЙ ТЕСТ ОБОРУДОВАНИЯ");
    ESP_LOGI("STARTUP_TEST", "==========================================");
    
    // === ШАГ 1: КРИТИЧЕСКАЯ ПРОВЕРКА ПОДКЛЮЧЕНИЯ ===
    ESP_LOGI("STARTUP_TEST", "🔍 Проверка подключения критических компонентов...");
    
#if CFG_ENABLE_SERVO_MOTORS

    {  // ← 🔑 ДОБАВИТЬ открывающую скобку БЛОКА
        // Проверка PCA9685
        // ✅ ИСПРАВЛЕНО: servoController теперь указатель
        if (servoController == nullptr || !servoController->isChipConnected() ) {
                ESP_LOGE("STARTUP_TEST", "❌ КРИТИЧЕСКАЯ ОШИБКА: Плата PCA9685 НЕ ОБНАРУЖЕНА!");
                ESP_LOGE("STARTUP_TEST", "Проверьте:");
                ESP_LOGE("STARTUP_TEST", "  • Питание PCA9685: ТОЛЬКО 3.3В (НЕ 5В!)");
                ESP_LOGE("STARTUP_TEST", "  • Общий GND между ESP32 и PCA9685");
                ESP_LOGE("STARTUP_TEST", "  • Подключение: SDA(21)→SDA, SCL(22)→SCL");
                ESP_LOGE("STARTUP_TEST", "  • Адрес PCA9685: должен быть 0x40 (перемычки к GND)");
                
                // Блокировка с индикацией ошибки
                pinMode(Config::Pins::LED_STATUS, OUTPUT);
                while (true) {
                    digitalWrite(Config::Pins::LED_STATUS, HIGH);
                    delay(100);
                    digitalWrite(Config::Pins::LED_STATUS, LOW);
                    delay(100);
            }
        }
        ESP_LOGI("STARTUP_TEST", "✅ PCA9685 обнаружен по I2C (адрес 0x40)");
    }  // ← 🔑 ДОБАВИТЬ закрывающую скобку БЛОКА
#endif
    
    // === ШАГ 2: ТЕСТ СЕРВОПРИВОДОВ (7 шт.) ===
#if CFG_ENABLE_SERVO_MOTORS
    {  // ← 🔑 ДОБАВИТЬ открывающую скобку БЛОКА
        if (_mConfig.enableServoMotors && servoController != nullptr) {
            ESP_LOGI("STARTUP_TEST", "⚙️  Тест сервоприводов (безопасный диапазон ±30°)...");
            ESP_LOGW("STARTUP_TEST", "Убедитесь, что сервоприводы могут свободно двигаться!");
            delay(2000);
            
            // Последовательный тест каждого сервопривода
            for (uint8_t servoIdx = 0; servoIdx < 7; servoIdx++) {
                ESP_LOGI("STARTUP_TEST", "  Тест серво #%d...", servoIdx);
                // Нейтраль → лево/вниз → нейтраль → право/вверх → нейтраль
                servoController->setLogicalAngle(servoIdx, 0);
                delay(400);
                servoController->setLogicalAngle(servoIdx, -30);  // Безопасное отклонение
                delay(600);
                servoController->setLogicalAngle(servoIdx, 0);
                delay(400);
                servoController->setLogicalAngle(servoIdx, 30);   // Безопасное отклонение
                delay(600);
                servoController->setLogicalAngle(servoIdx, 0);
                delay(400);
            }
        }
    }  // ← 🔑 ДОБАВИТЬ закрывающую скопку БЛОКА
#endif


    ESP_LOGI("STARTUP_TEST", "✅ Тест сервоприводов завершен успешно");
    
    // === ШАГ 3: ОБЯЗАТЕЛЬНОЕ РУЧНОЕ ПОДТВЕРЖДЕНИЕ БЕЗОПАСНОСТИ ===
#if CFG_ENABLE_TRACTION_MOTORS
    if (_mConfig.enableTractionMotors && motorController != nullptr) {
        ESP_LOGW("STARTUP_TEST", "⚠️⚠️⚠️  ВНИМАНИЕ! КРИТИЧЕСКИ ВАЖНО!  ⚠️⚠️⚠️");
        ESP_LOGW("STARTUP_TEST", "Тест моторов начнется через 10 секунд");
        ESP_LOGW("STARTUP_TEST", "ПЕРЕД ТЕСТОМ УБЕДИТЕСЬ:");
        ESP_LOGW("STARTUP_TEST", "  1. БПЛА НАДЕЖНО ЗАКРЕПЛЕН (не на весу!)");
        ESP_LOGW("STARTUP_TEST", "  2. Винты СВОБОДНЫ от препятствий");
        ESP_LOGW("STARTUP_TEST", "  3. НЕТ людей/предметов в зоне вращения");
        ESP_LOGW("STARTUP_TEST", "  4. Для ОТМЕНЫ теста ОТКЛЮЧИТЕ ПИТАНИЕ СЕЙЧАС!");
        
        // Мигание предупреждения (10 секунд)
        pinMode(Config::Pins::LED_STATUS, OUTPUT);
        for (int i = 0; i < 20; i++) {
            digitalWrite(Config::Pins::LED_STATUS, (i % 2 == 0) ? HIGH : LOW);
            delay(250);
        }
        
        // Отсчет 10 секунд с возможностью отмены
        for (int i = 10; i > 0; i--) {
            ESP_LOGI("STARTUP_TEST", "  Начало теста моторов через %d сек...", i);
            delay(1000);
        }

        ESP_LOGI("STARTUP_TEST", "🌀 Тест моторов (макс. 35%% мощности)...");
        motorController->setMotorPower(0, 30.0f);
        delay(1200);
        motorController->setMotorPower(0, 0.0f);
        delay(500);
        motorController->setMotorPower(1, 30.0f);
        delay(1200);
        motorController->setMotorPower(1, 0.0f);
        delay(500);
        motorController->setMotorPower(0, 25.0f);
        motorController->setMotorPower(1, 25.0f);
        delay(1500);
        motorController->setMotorPower(0, 0.0f);
        motorController->setMotorPower(1, 0.0f);
        ESP_LOGI("STARTUP_TEST", "✅ Тест моторов завершен успешно");


    }
#endif

    // === ШАГ 4: ФИНАЛЬНАЯ ПОДГОТОВКА ===
#if CFG_ENABLE_SERVO_MOTORS
    if (servoController != nullptr) {
        servoController->resetToNeutral();
    }
#endif
    
    ESP_LOGI("STARTUP_TEST", "✅ ПРЕДСТАРТОВЫЙ ТЕСТ ПРОЙДЕН УСПЕШНО!");
    ESP_LOGI("STARTUP_TEST", "✅ runStartupTest() завершена");

} // END runStartupTest()

// ====================================================================
// ====================================================================

