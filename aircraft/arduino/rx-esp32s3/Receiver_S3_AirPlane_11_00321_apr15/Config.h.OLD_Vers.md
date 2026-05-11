/*
// НОВОЕ Распределение пинов 
//==================================================================
//  🔧 РАБОЧИЙ Исправленный Config.h       
//==================================================================


Отличное решение! Вернемся к практической задаче. 
Разберем подключение LoRa модуля RA-01 SX1278 - это действительно важный момент. 📡
🔌 Подключение LoRa модуля RA-01 SX1278
SPI интерфейс - что это:

SPI (Serial Peripheral Interface) - это синхронный последовательный интерфейс, где есть "ведущий" (ESP32) и "ведомый" (LoRa модуль).
Расшифровка выводов LoRa:
Вывод LoRa	Назначение	Подключение к ESP32
SCK	Serial Clock - тактовый сигнал	GPIO18
MISO	Master In Slave Out - данные от LoRa к ESP32	GPIO19
MOSI	Master Out Slave In - данные от ESP32 к LoRa	GPIO23
NSS/SS	Slave Select - выбор устройства	GPIO5
RST	Reset - сброс модуля	GPIO13
DIO0	Digital I/O 0 - прерывания	GPIO4
✅ Ваша конфигурация ПРАВИЛЬНАЯ:


📋 Полная схема подключения ESP32:
ESP32 Dev Module (38 pin) → Внешние модули
┌────────────-─────┬─────────────────────────┐
│ ESP32 Pin        │ Подключение             │
├───────────-──────┼─────────────────────────┤
│ GPIO 5           │ LoRa SS/NSS             │
│ GPIO 13          │ LoRa RST                │
│ GPIO 4           │ LoRa DIO0               │
│ GPIO 18          │ LoRa SCK                │
│ GPIO 19          │ LoRa MISO               │
│ GPIO 23          │ LoRa MOSI               │
│ GPIO 16          │ GPS TX → ESP32 RX-16    │
│ GPIO 17          │ GPS RX ← ESP32 TX-17    │
│ GPIO 21          │ I2C SDA (общий)         │
│ GPIO 22          │ I2C SCL (общий)         │
│ GPIO 25,26,27,   │ Сервоприводы (PWM)      │
│ GPIO 15,33       │ Сервоприводы (PWM)      │
│ GPIO 32,14       │ Моторы (PWM)            │
└───────────=──────┴─────────────────────────┘
//  🎯 Полностью исправленный
//
//## 📌 Критически важные замечания по пинам (ваше требование соблюдено!)
//
//Все пины остаются **без изменений** согласно `Config.h`:
//
// SERVOS: через PCA9685 (I2C: SDA=21, SCL=22)
//   Серво #0 → Канал 0: Левый элерон
//   Серво #1 → Канал 1: Правый элерон  
//   Серво #2 → Канал 2: Левый руль высоты
//   Серво #3 → Канал 3: Правый руль высоты
//   Серво #4 → Канал 4: Руль направления
//   Серво #5 → Канал 5: Парашют
//   Серво #6 → Канал 6: Вспомогательный
*/
/**
* @file Config.h
* @brief Конфигурация проекта ESP32 Receiver
* @version 5.2.2 - Добавлена поддержка конфигураций АКБ 3S-12S
* @date 2026
*/
/**
* @file Config.h
* @brief Конфигурация проекта ESP32 Receiver
* @version 5.2.3 - Исправлен конфликт имён BatteryMonitor
* @date 2026
*/
/**
 * @file Config.h
 * @brief Конфигурация проекта ESP32 Receiver
 * @version 6.6.1 - Исправлен конфликт BatteryMonitor, обновлён ADC драйвер
 * @date 2026
 * 
 * @note ВСЕ ПИНЫ ЗАФИКСИРОВАНЫ - не менять без серьёзной причины!
 */
/**
 * @file Config.h
 * @brief Позиция: внутри #if defined(CONFIG_IDF_TARGET_ESP32S3)
 * @version 7.0.1 - Исправлен конфликт BatteryMonitor, обновлён ADC драйвер
 * @date 2026-04-18
 * 
 * @note Описание: Исправление конфликта пинов SPI для LoRa на ESP32-S3
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <SPI.h>
#include <Arduino.h>
#include <RadioLib.h>
#include <cstdint>
#include <driver/ledc.h>

// ✅ НОВЫЙ ADC драйвер (ESP-IDF 5.0+)
#include <esp_adc/adc_oneshot.h> // ESP-IDF 5.0+ ADC API

#pragma once

// Версия системы
#define SYSTEM_VERSION "7.0.1"
#define BUILD_DATE "18-04-2026"

// ============================================================================
// НАСТРОЙКИ ПРОЕКТА (ESP32-S3)
// ============================================================================
// ============================================================================
// 🔋 КОНФИГУРАЦИЯ ПРОВЕРКИ НАПРЯЖЕНИЯ АКУМУЛЯТОРНОЙ БАТАРЕИ
// ============================================================================
/**
 * @brief Константы мониторинга батареи
 * @note Убран namespace для избежания конфликта с классом BatteryMonitor
 */
 /* 
namespace BattAkkMonitor {

//  ****************************       
    // Для совместимости оставляем 34, но BatteryMonitor обработает это динамически
    //      ADC1-4, TOUCH5, GPIO5, PIN(18),Pic(4);
    // A4	ADC1_4	        5	                YES
    // constexpr uint8_t BATTERY_ADC_PIN = 7; 
    constexpr uint8_t BATTERY_ADC_PIN = 7;              ///< GPIO34 для измерения напряжения
    // constexpr uint8_t BATTERY_ADC_PIN = 34;          ///< GPIO34 для измерения напряжения
    constexpr float BATTERY_ADC_MAX_VOLTAGE = 3.3f;     ///< Макс. напряжение ADC ESP32
    constexpr int BATTERY_ADC_RESOLUTION = 4095;        ///< 12-bit ADC (0-4095)

    // ✅ ИСПРАВЛЕНО: ADC_ATTEN_DB_12 вместо устаревшего ADC_ATTEN_DB_11
    constexpr adc_atten_t BATTERY_ADC_ATTENUATION = ADC_ATTEN_DB_12;  ///< Затухание для 0-3.3В
    
    // Коэффициент делителя: (R1 + R2) / R2 = (100к + 12к) / 12к = 9.33
    // Коэффициент делителя: (R1 + R2) / R2 = (100к + 10к) / 10к = 9.0
    //constexpr float VOLTAGE_DIVIDER_RATIO = 9.0f;

    // Коэффициент делителя напряжения (подобрать под вашу схему!)
    // Формула: V_battery = V_adc * (R1 + R2) / R2
    // Пример: R1=100к, R2=12к → ratio = 9.33
    // Пример: R1=100к, R2=10к → ratio = 9.0
    // constexpr float VOLTAGE_DIVIDER_RATIO = 9.33f;
    // Коэффициент делителя напряжения: (R1+R2)/R2 = (100к+10к)/10к = 9.0
    //constexpr float BATTERY_VOLTAGE_DIVIDER_RATIO = 9.0f;

    // ✅ ИСПРАВЛЕНО (под ваше железо):
    // 🔧 ВАЖНО: Измерьте фактические значения резисторов!
    // Формула: ratio = (R1 + R2) / R2
    // Пример: R1=100кΩ, R2=10кΩ → ratio = 11.0
    // Пример: R1=100кΩ, R2=12кΩ → ratio = 9.33
    // 
    // Для коррекции: измерьте V_battery вольтметром и V_adc на GPIO34,
    // затем: ratio = V_battery / V_adc_measured
    //
    // ✅ ВРЕМЕННОЕ ЗНАЧЕНИЕ ПО КАЛИБРОВКЕ (16.8В / 13.64В = 1.232):
    constexpr float BATTERY_VOLTAGE_DIVIDER_RATIO = 11.1f;  // 🔑 ИЗМЕНИТЬ ПОД ВАШУ СХЕМУ!


    // Конфигурация АКБ по умолчанию (изменяется через EEPROM или команду)
    // constexpr uint8_t DEFAULT_CELL_COUNT = 6;    ///< 6S по умолчанию
    constexpr uint8_t DEFAULT_CELL_COUNT = 4;    ///< 4S по умолчанию
    
    // Глобальные константы (без префикса BATTERY_)
    // Пороговые значения (будут пересчитаны на основе cellCount)
    constexpr float VOLTAGE_PER_CELL_MAX = 4.20f;     ///< Макс на ячейку
    constexpr float VOLTAGE_PER_CELL_NOMINAL = 3.70f; ///< Номинал на ячейку
    constexpr float VOLTAGE_PER_CELL_MIN = 3.00f;     ///< Мин на ячейку
    constexpr float VOLTAGE_PER_CELL_CRITICAL = 2.80f;///< Крит на ячейку
    
    
    // Интервалы проверки
    constexpr uint32_t CHECK_INTERVAL_MS = 5000;      ///< Проверка каждые 5 сек
    constexpr uint32_t STARTUP_CHECK_DELAY = 2000;    ///< Задержка перед первой проверкой
    constexpr uint32_t CRITICAL_ACTION_DELAY = 1000;  ///< Задержка перед аварийным действием

    
    //     {4,  16.8f, 14.8f, 12.0f, 11.2f, "4S LiPo"},
    // Пороговые значения напряжения
    constexpr float VOLTAGE_MAX = 16.8f;         // Максимальное напряжение (перезаряд)
    constexpr float VOLTAGE_NOMINAL = 14.8f;
    constexpr float VOLTAGE_MIN = 12.0f;         // Минимальное напряжение (разряд)
    constexpr float VOLTAGE_CRITICAL = 11.2f;    // Критическое напряжение
    
}; // END namespace BattAkkMonitor
*/



// ================== КОНФИГУРАЦИЯ МОДУЛЕЙ ==================
//
//==================================================================================
// 🔧 КОНФИГУРАЦИЯ КОМПИЛЯЦИИ (Preprocessor)
//==================================================================================
// 🔧 Эти макросы управляют включением кода на этапе компиляции (preprocessor)
// Изменение этих флагов требует перекомпиляции проекта!
//  Проект уже использует двухуровневую систему управления модулями:
//  ### 1. Управление на этапе компиляции (preprocessor)
#define CFG_ENABLE_SERVO_MOTORS    1   ///< 1 = Включить код сервоприводов (PCA9685)
#define CFG_ENABLE_TRACTION_MOTORS 1   ///< 1 = Включить код тяговых моторов (LEDC)
#define CFG_ENABLE_RADIO           1   ///< 1 = Включить код радиомодуля (LoRa SX1278)
#define CFG_ENABLE_IMU             1   ///< 1 = Включить код инерциального модуля (GY-87)
#define CFG_ENABLE_SD              0   ///< 1 = Включить поддержку SD карты
#define CFG_ENABLE_ESP32_CAM       0   ///< 1 = Включить поддержку ESP32-CAM
#define CFG_ENABLE_GPS             0   ///< 1 = Включить поддержку GPS (UART)
#define CFG_ENABLE_I2C_MASTER      1   ///< 1 = Включить нативный драйвер I2C
#define CFG_ENABLE_BATTERY_MONITOR 0   ///< 1 = Включить мониторинг батареи


//==================================================================================
// 🔧 КОНФИГУРАЦИЯ МОДУЛЕЙ (Runtime)
//==================================================================================
// Структура для управления логикой во время выполнения
/**
* @struct ModuleConfig
* @brief Структура для управления логикой во время выполнения
* @note Позволяет динамически включать/выключать модули без перекомпиляции
*/
// ### 2. Управление во время выполнения (runtime)
struct ModuleConfig {
    bool enableServoMotors;   ///< Включить поддержку сервоприводов
    bool enableTractionMotors;///< Включить поддержку тяговых моторов
    bool enableRadio;         ///< Включить поддержку радиомодуля (LoRa)
    bool enableIMU;           ///< Включить поддержку IMU
    bool enableSD;            ///< Включить поддержку SD карты
    bool enableESP32Cam;      ///< Включить поддержку ESP32-CAM
    bool enableGPS;           ///< Включить поддержку GPS
    bool enableI2CMaster;     ///< Включить поддержку I2C Master
    bool enableBatteryMonitor;///< Включить поддержку BatteryMonitor Master
};

// Инициализация структуры на основе макросов компиляции
//  ### 2. Управление во время выполнения (runtime)
static const ModuleConfig _defaultModuleConfig = {
    .enableServoMotors    = (CFG_ENABLE_SERVO_MOTORS == 1),
    .enableTractionMotors = (CFG_ENABLE_TRACTION_MOTORS == 1),
    .enableRadio          = (CFG_ENABLE_RADIO == 1),
    .enableIMU            = (CFG_ENABLE_IMU == 1),
    .enableSD             = (CFG_ENABLE_SD == 1),
    .enableESP32Cam       = (CFG_ENABLE_ESP32_CAM == 1),
    .enableGPS            = (CFG_ENABLE_GPS == 1),
    .enableI2CMaster      = (CFG_ENABLE_I2C_MASTER == 1),
    .enableBatteryMonitor = (CFG_ENABLE_BATTERY_MONITOR == 1)
};

extern const ModuleConfig _mConfig;


// ============================================================================
// КОНСТАНТЫ
// ============================================================================
// Конфигурация вынесена в отдельный namespace для лучшей организации
/**
 * @brief Адаптивная и безопасная конфигурация пинов для ESP32-S3
 * @details Исправлены конфликты:
 *   • Разделены LORA_CS и VSPI_CS (были оба на 39)
 *   • BATTERY_ADC_PIN перенесен на GPIO7 (ADC1_CH6) для обхода конфликта с USB D- (GPIO19/20)
 *   • Все пины проверены на отсутствие конфликтов с Flash, PSRAM и USB CDC
 */
namespace Config {
    // В E49_Config.h, в namespace Config или отдельно
    // extern SPIClass* hspi; // Объявляем, определим в .cpp файле

    namespace Pins {
//=============================================================================
// 🔧 ESP32-S3 PIN MAP — АДАПТИВНАЯ КОНФИГУРАЦИЯ
//    КАРТА ПИНОВ (ESP32-S3) 
//=============================================================================
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_USB_CDC_ON_BOOT)
    // ================== ESP32-S3 PIN MAP (БЕЗОПАСНАЯ) ==================
    // ⚠️ ИЗБЕГАТЬ: 
    //   • GPIO0 (boot strapping)
    //   • GPIO19/20 (USB CDC - если используется)
    //   • GPIO45-48 (Flash/PSRAM - КРИТИЧЕСКИ!)
    //   • GPIO21 (обычно стрепинг)

    // 📡 LoRa SX1278 (SPI через FSPI)
    // Используем фиксированные пины FSPI для максимальной производительности
    // 🔌 FSPI шина (стандарт для ESP32-S3)
    constexpr uint8_t LORA_CS    = 10; // FSPI CS-NSS
    constexpr uint8_t FSPI_SCLK  = 12; // FSPI Clock
    constexpr uint8_t FSPI_MISO  = 13; // FSPI Master In Slave Out
    constexpr uint8_t FSPI_MOSI  = 11; // FSPI Master Out Slave In

    // --- LoRa Control GPIOs ---
    // ВНИМАНИЕ: GPIO 9 занят под I2C SCL. Пин RST перенесен на 15 (безопасный GPIO).
    constexpr uint8_t LORA_RST   = 15; // Gpio_16
    constexpr uint8_t LORA_DIO0  = 4;  // Прерывание приема пакета
    constexpr uint8_t LORA_DIO1  = 14; // Прерывание таймаута

    // --- I2C Шина (Общая для MPU9250, PCA9685) ---
    constexpr uint8_t I2C_SDA    = 8;
    constexpr uint8_t I2C_SCL    = 9; // Конфликт с RST(9) устранен (RST -> 15)

                       
    // ⚡ Моторы через LEDC (аппаратный ШИМ)
    // 🔑 Проверка: пины 40,41 поддерживают LEDC на ESP32-S3
    constexpr uint8_t MOTOR_PIN_1 = 40;
    constexpr uint8_t MOTOR_PIN_2 = 41;
    constexpr uint8_t motorPins[2] = {40, 41};
    constexpr ledc_channel_t motorChannels[2] = {LEDC_CHANNEL_1, LEDC_CHANNEL_2};

    // --- Мониторинг Батареи (ADC) ---
    // GPIO 7 = ADC1_CH6. Поддерживает калибровку (Line Fitting).
    // Безопасный пин, не конфликтует с USB (19/20) и Flash/PSRAM.
    constexpr uint8_t BATTERY_ADC_PIN = 7;

    // --- GPS (UART) ---
    constexpr uint8_t GPS_RX = 18; // Прием данных от GPS
    constexpr uint8_t GPS_TX = 17; // Передача данных GPS

    // --- Периферия ---
    constexpr uint8_t BUZZER_PIN = 16;
    constexpr uint8_t LED_STATUS = 2; // Встроенный LED

    // ============================================================================
    // 🔗 UART ШИНА ДЛЯ RASPBERRY PI ZERO 2W (UART2)
    // ============================================================================
    /**
    * @brief Аппаратный UART2 для связи с RPi Zero 2W
    * @details 
    *   • TX (ESP32-S3 GPIO5) → RX (RPi GPIO14/BCM14)
    *   • RX (ESP32-S3 GPIO6) ← TX (RPi GPIO15/BCM15)
    *   • Скорость: 115200 или 921600 бод (аппаратно стабильно)
    *   • Логика 3.3V, общий GND обязателен. Уровневые конвертеры НЕ нужны.
    *   • RTS (GPIO3) → RPi CTS (ESP32 сообщает, что готов принять)
    *   • CTS (GPIO42)← RPi RTS (ESP32 получает сигнал о готовности RPi принять)
    *   • Скорость: 921600 бод. Без Flow Control при такой скорости возможны потери пакетов.
    * @note GPIO 5 и 6 безопасны на ESP32-S3-WROOM-1, не конфликтуют с Flash/PSRAM/USB.
    */
    constexpr uint8_t UART_RPI_TX  = 5;  ///< ESP32-S3 TX → RPi RX
    constexpr uint8_t UART_RPI_RX  = 6;  ///< ESP32-S3 RX ← RPi TX
    constexpr uint8_t UART_RPI_RTS = 3;  ///< Ready To Send (ESP32)
    constexpr uint8_t UART_RPI_CTS = 42; ///< Clear To Send (RPi)    
    // Опционально: аппаратный flow control (раскомментировать при высоких нагрузках)
    // constexpr uint8_t UART_RPI_RTS = 3;  ///< Ready To Send (ESP32)
    // constexpr uint8_t UART_RPI_CTS = 42; ///< Clear To Send (RPi)

#else
        // ================== ESP32 ORIGINAL PIN MAP ==================
        // ================== КЛАССИЧЕСКИЙ ESP32 (оригинальные пины) ==================
        constexpr uint8_t LORA_CS    = 5;
        constexpr uint8_t LORA_RST   = 13;
        constexpr uint8_t LORA_DIO0  = 4;
        constexpr uint8_t LORA_DIO1  = 14;
        constexpr uint8_t VSPI_SCLK  = 18;
        constexpr uint8_t VSPI_MISO  = 19;
        constexpr uint8_t VSPI_MOSI  = 23;
        constexpr uint8_t I2C_SDA    = 21;
        constexpr uint8_t I2C_SCL    = 22;
        constexpr uint8_t motorPins[2] = {32, 33};
        constexpr uint8_t GPS_RX     = 16;
        constexpr uint8_t GPS_TX     = 17;
        constexpr uint8_t BUZZER_PIN = 12;
        constexpr uint8_t LED_STATUS = 2;
        constexpr uint8_t BATTERY_ADC_PIN = 34;

        /*  
        // ============================================================================
        // 📡 LoRa МОДУЛЬ (Ra-01 SX1278) - SPI!
        // ============================================================================
        constexpr uint8_t LORA_CS = 5;    // Chip Select
        constexpr uint8_t LORA_RST = 13;  // ✅ БЕЗОПАСНЫЙ Reset
        constexpr uint8_t LORA_DIO0 = 4;  // Digital IO 0 (прерывания/пакет готов)
        constexpr uint8_t LORA_DIO1 = 14; // 🆕 Digital IO 1 (Rx Timeout / CAD для RadioLib 7.6.0)
        // SPI: SCK=18, MISO=19, MOSI=23 (общие с VSPI)

        // ============================================================================
        // 🔌 SPI ПЕРИФЕРИЯ (VSPI - ВЫСОКАЯ ПРОИЗВОДИТЕЛЬНОСТЬ)
        // ============================================================================
        // Интерфейс	MOSI	MISO	CLK	    CS
        // HSPI  SPI3	11	    13	    12	    10
        // HSPI (SPI2)	GPIO 11	GPIO 13	GPIO 12	GPIO 10
        // VSPI (SPI3)	GPIO 35	GPIO 37	GPIO 36	GPIO 39

        constexpr uint8_t VSPI_SCLK = 36;       // ✅ Стандартный SCLK для VSPI
        constexpr uint8_t VSPI_CS = 39;         // ✅ Стандартный CS для VSPI  
        constexpr uint8_t VSPI_MISO  = 37;      // ✅ Стандартный MISO  для VSPI  
        constexpr uint8_t VSPI_MOSI = 35;       // ✅ Стандартный MOSI для VSPI
        //=============================================================================
        constexpr uint8_t VSPI_SCLK = 18;     // ✅ Стандартный SCLK для VSPI
        constexpr uint8_t VSPI_MISO = 19;     // ✅ Стандартный MISO для VSPI  
        constexpr uint8_t VSPI_MISO  = 37; // 🔑 ЗАМЕНЕНО: 19 → 37 (безопасно от USB)
        constexpr uint8_t VSPI_MOSI = 23;     // ✅ Стандартный MOSI для VSPI

        // ============================================================================
        // 🤖 МОТОРЫ Без  СЕРВОПРИВОДОВ
        // ============================================================================
        constexpr uint8_t motorPins[2] = {32, 33};
        // Каналы LEDC для моторов  
        constexpr ledc_channel_t motorChannels[2] = {
            LEDC_CHANNEL_5,
            LEDC_CHANNEL_6
        };

        // ============================================================================
        // 🛰️ GPS МОДУЛЬ (ATGM336H) - UART
        // ============================================================================
        //  Порт	TX	    RX	    Примечания
        // UART0	GPIO 43	GPIO 44	Жестко привязан к USB-Serial, не изменяется.
        // UART1	GPIO 17	GPIO 18	Может быть переназначен.
        constexpr uint8_t GPS_RX = 18;    // ESP32 RX ← GPS TX  
        constexpr uint8_t GPS_TX = 17;    // ESP32 TX → GPS RX
        //=============================================================================
        // constexpr uint8_t GPS_RX = 16;    // ESP32 RX ← GPS TX  
        // constexpr uint8_t GPS_TX = 17;    // ESP32 TX → GPS RX

        // ============================================================================
        // 🧭 IMU МОДУЛЬ (GY-87) - I2C
        // ============================================================================
        // GPIO 8 (SDA) - Линия данных
        // GPIO 9 (SCL) - Линия тактирования
        constexpr uint8_t I2C_SDA = 8;
        constexpr uint8_t I2C_SCL = 9;
        //=============================================================================
        // constexpr uint8_t I2C_SDA = 21;
        // constexpr uint8_t I2C_SCL = 22;

        // ============================================================================
        // 🔗 СВЯЗЬ С ESP32-CAM (через I2C - тот же BUS!)
        // ============================================================================
        constexpr uint8_t CAM_I2C_ADDRESS = 0x30;
        */

        // ============================================================================
        // 💾 ДОПОЛНИТЕЛЬНЫЕ УСТРОЙСТВА И РЕЗЕРВ
        // ============================================================================
        // constexpr uint8_t BUZZER_PIN = 13;      // ✅ Звуковая сигнализация (только вход)
        // constexpr uint8_t BUZZER_PIN = 12;      // 🔑 ЗАМЕНЕНО: 13 → 12 (устранен конфликт с RST)
        // constexpr uint8_t LED_STATUS = 2;       // ✅ Индикация состояния (только вход)
        // constexpr uint8_t RESERVE_1 = 36;       // ✅ Резерв (только вход, аналоговый)
        // constexpr uint8_t RESERVE_2 = 39;       // ✅ Резерв (только вход, аналоговый)
#endif
    }; // end namespace Pins

    // Config::Battery::
    // =========================================================================
    // ПАРАМЕТРЫ БАТАРЕИ (Battery Monitor)
    // =========================================================================
    namespace Battery {
        // Ссылка на пин из раздела Pins для удобства
        constexpr uint8_t ADC_PIN = Pins::BATTERY_ADC_PIN;

        constexpr float ADC_MAX_VOLTAGE = 3.3f;          // Макс. напряжение входа ADC
        constexpr int ADC_RESOLUTION = 4095;             // 12-бит (0-4095)
        constexpr adc_atten_t ADC_ATTENUATION = ADC_ATTEN_DB_12; // Ослабление для 0-3.3В
        
        constexpr float VOLTAGE_DIVIDER_RATIO = 11.1f;   // Коэффициент делителя напряжения
        constexpr uint8_t DEFAULT_CELL_COUNT = 4;        // АКБ по умолчанию (4S)

        constexpr uint32_t CHECK_INTERVAL_MS = 5000;     // Интервал проверки (5 сек)
        constexpr uint32_t STARTUP_CHECK_DELAY = 2000;   // Задержка перед первым стартом (2 сек)

        // Коэффициент делителя напряжения (подобрать под вашу схему!)
        // Формула: V_battery = V_adc * (R1 + R2) / R2
        // Пример: R1=100к, R2=12к → ratio = 9.33
        // Пример: R1=100к, R2=10к → ratio = 9.0
        // constexpr float VOLTAGE_DIVIDER_RATIO = 9.33f;
        // Коэффициент делителя напряжения (подобрать под вашу схему!)
        // Формула: V_battery = V_adc * (R1 + R2) / R2
        // Пример: R1=100к, R2=12к → ratio = 9.33
        // Пример: R1=100к, R2=10к → ratio = 9.0
        // constexpr float VOLTAGE_DIVIDER_RATIO = 9.33f;
        // Коэффициент делителя напряжения: (R1+R2)/R2 = (100к+10к)/10к = 9.0
        constexpr float BATTERY_VOLTAGE_DIVIDER_RATIO = 9.0f;

        constexpr uint8_t BATTERY_ADC_PIN = 7;              ///< GPIO34 для измерения напряжения
        // constexpr uint8_t BATTERY_ADC_PIN = 34;          ///< GPIO34 для измерения напряжения
        constexpr float BATTERY_ADC_MAX_VOLTAGE = 3.3f;     ///< Макс. напряжение ADC ESP32
        constexpr int BATTERY_ADC_RESOLUTION = 4095;        ///< 12-bit ADC (0-4095)

        // ✅ ИСПРАВЛЕНО: ADC_ATTEN_DB_12 вместо устаревшего ADC_ATTEN_DB_11
        constexpr adc_atten_t BATTERY_ADC_ATTENUATION = ADC_ATTEN_DB_12;  ///< Затухание для 0-3.3В

    }; // END namespace Battery



        // --- I2C Master Configuration ---
        // ============================================================================
        // 🔗 СВЯЗЬ С ESP32-CAM  I2C Master Configuration (через I2C - тот же BUS!)
        // ============================================================================
    namespace I2C {
        // constexpr uint8_t SDA_PIN       = 8;           // SDA pin for I2C master
        // constexpr uint8_t SCL_PIN       = 9;           // SCL pin for I2C master
        //=============================================================================
        //constexpr uint8_t SDA_PIN       = 21;           // SDA pin for I2C master
        //constexpr uint8_t SCL_PIN       = 22;           // SCL pin for I2C master
        constexpr uint32_t FREQUENCY    = 400000;    // I2C frequency (400kHz standard, 1000000 fast possible)
        constexpr uint32_t TIMEOUT_MS   = 1000;     // Timeout for I2C operations
    }; // namespace I2C

    // --- PCA9685 Configuration ---
    namespace PCA9685 {
        constexpr uint8_t DEFAULT_ADDRESS = 0x40; // Standard address (can be changed by A0-A5 pins)
        //constexpr uint16_t PWM_FREQ = 50;         // Typical servo frequency in Hz
        constexpr uint8_t SERVO_COUNT = 7;        // Number of servos controlled
        // Define which PCA9685 channels correspond to which function

            namespace Channels {
                constexpr uint8_t SERVO_ELEVATOR_LEFT = 0;  // Channel 0
                constexpr uint8_t SERVO_ELEVATOR_RIGHT = 1; // Channel 1
                constexpr uint8_t SERVO_RUDDER_LEFT = 2;    // Channel 2
                constexpr uint8_t SERVO_RUDDER_RIGHT = 3;   // Channel 3
                constexpr uint8_t SERVO_PARACHUTE = 4;      // Channel 4
                constexpr uint8_t SERVO_AUX_1 = 5;          // Channel 5
                constexpr uint8_t SERVO_AUX_2 = 6;          // Channel 6
            }; // END Channels 
    }; // END  namespace PCA9685

                        /*
                                Мы имеем модуль GPS ATGM336H, который использует UART для связи.
                        В UART соединении всегда используется перекрестное подключение:
                            RX (приемник) одного устройства подключается к TX (передатчику) другого.
                            TX (передатчик) одного устройства подключается к RX (приемнику) другого.
                        В коде указано:
                        constexpr uint8_t GPS_RX = 16; // ESP32 RX ← GPS TX
                        constexpr uint8_t GPS_TX = 17; // ESP32 TX → GPS RX
                        Это означает:
                            Пин GPS_RX (16 на ESP32) - это пин, на котором ESP32 принимает данные (RX), 
                        поэтому он должен быть подключен к пину TX модуля GPS.
                            Пин GPS_TX (17 на ESP32) - это пин, на котором ESP32 передает данные (TX), 
                        поэтому он должен быть подключен к пину RX модуля GPS.
                        Таким образом, в коде указано правильно: перекрестное подключение.

                        Однако, обратите внимание на комментарии:
                        "ESP32 RX ← GPS TX" - это правильно: ESP32 принимает (RX) данные от передатчика (TX) GPS.
                        "ESP32 TX → GPS RX" - это правильно: ESP32 передает (TX) данные на приемник (RX) GPS.

                        Итак, подключение должно быть:
                        ESP32 pin 16 (RX) -> GPS module TX
                        ESP32 pin 17 (TX) -> GPS module RX
                        Проверьте распиновку вашего модуля ATGM336H, чтобы убедиться, что пины TX и RX на модуле подписаны правильно.
                        Обычно на модулях GPS:
                            Пин TX модуля GPS - это выход данных (передает данные) и должен быть подключен к RX ESP32.
                            Пин RX модуля GPS - это вход данных (принимает команды) и должен быть подключен к TX ESP32.
                        Вы все правильно указали!
                        Отличный вопрос! Вы абсолютно правы - нужно ПЕРЕКРЕСТНОЕ подключение! 🔄
                        🔌 Правильное подключение UART:
                        Ваша конфигурация - ВЕРНА:
                            constexpr uint8_t GPS_RX = 16;    // ESP32 RX ← GPS TX  
                            constexpr uint8_t GPS_TX = 17;    // ESP32 TX → GPS RX
                        📋 Наглядная схема подключения:

                        ESP32 Dev Module        ATGM336H GPS Module
                        ┌────────────────┐      ┌──────────────────┐
                        │                │      │                  │
                        │   GPIO17 (TX) ───────→│ RX (Input)       │
                        │                │      │                  │
                        │   GPIO16 (RX) ←───────│ TX (Output)      │
                        │                │      │                  │
                        │   GND         ────────│ GND              │
                        │                │      │                  │
                        │   3.3V        ────────│ VCC              │
                        └────────────────┘      └──────────────────┘

                        🎯 Простое правило:
                            TX (Transmit) → RX (Receive) - передатчик к приемнику
                            RX (Receive) ← TX (Transmit) - приемник к передатчику
                        🔍 Проверка для вашего кода:
                            ✅ GPS_RX = 16 - ESP32 принимает данные на pin 16 от GPS TX
                            ✅ GPS_TX = 17 - ESP32 передает данные с pin 17 на GPS RX
                        💡 Дополнительно для ATGM336H:
                            PPS pin (если есть) - для точного времени, можно подключить к любому GPIO
                            VCC - 3.3V (не 5V!)
                            GND - общая земля
                        Ваша схема подключения корректна! Если GPS не заработает с первой попытки, проверьте:
                            Правильность подключения проводов
                            Настройки UART в коде (скорость 9600 для ATGM336H)
                            Питание 3.3V на модуле
                            */
                                    // ============================================================================
                                    // 🧭 IMU МОДУЛЬ (GY-87) - I2C
                                    // ============================================================================
                                    //constexpr uint8_t I2C_SDA = 21;
                                    //constexpr uint8_t I2C_SCL = 22;

                                    // ============================================================================
                                    // 🤖 МОТОРЫ И СЕРВОПРИВОДЫ
                                    // ============================================================================
                                    //const uint8_t MOTOR_A = 32;
                                    //const uint8_t MOTOR_B = 33;   // ✅ GPIO14 вместо 33
                                    //const int motorPins[2] = {32, 33};

                                    // ============================================================================
                                    // 🔗 СВЯЗЬ С ESP32-CAM (через I2C - тот же BUS!)
                                    // ============================================================================
                                    //constexpr uint8_t CAM_I2C_ADDRESS = 0x30;

                                    //    enum ServoAssignments {
                                    //        SERVO_LEFT_WING = 0,    // 27  Левое крыло (элерон) - пин 25
                                    //        SERVO_RIGHT_WING = 1,   // 33  Правое крыло (элерон) - пин 26  
                                    //        SERVO_LEFT_RUDDER = 2,  // 25  Левый руль направления (хвост) - пин 27
                                    //        SERVO_RIGHT_RUDDER = 3, // 26  Правый руль направления (хвост) - пин 33
                                    //        SERVO_PARACHUTE = 4     // 15  Парашют - пин 15
                                    //    };

    // =========================================================================
    // ПАРАМЕТРЫ РАДИО (LoRa)
    // =========================================================================
    namespace Radio {
        // ============================================================================
        // 🔧 ФЛАГИ УСЛОВНОЙ КОМПИЛЯЦИИ LoRa
        // ============================================================================
        /**
        * @brief Флаг поддержки DIO1 для SX1278
        * @details Установите в 1 для модулей с DIO1 (RadioLib 7.6.0+)
        *          Установите в 0 для старых модулей (только DIO0)
        * @note Все пины остаются зафиксированными в Config::Pins
        */
        #ifndef LORA_ENABLE_DIO1
        #define LORA_ENABLE_DIO1 1  // 🔑 ИЗМЕНИТЬ: 1 => с DIO1, 0 = только DIO0
        #endif

        // ============================================================================
        // 🔧 АВТОПРОВЕРКА КОНФИГУРАЦИИ (compile-time assert)
        // ============================================================================
        // ✅ СТАЛО (ИСПРАВЛЕНО):
        #if LORA_ENABLE_DIO1
            // 🔑 static_assert выполняется на этапе компиляции C++ и корректно 
            //    проверяет constexpr константы из namespace Config
            static_assert(Config::Pins::LORA_DIO1 != RADIOLIB_NC,
                    "❌ ОШИБКА: LORA_ENABLE_DIO1=1, но Config::Pins::LORA_DIO1 не задан (равен RADIOLIB_NC)!");
        #endif        
        //====================================================

        constexpr uint16_t TRANSCEIVER_DELAY = 850;
        constexpr uint16_t CONNECTION_CHECK_INTERVAL = 10000;
        constexpr uint16_t DATA_SEND_INTERVAL = 1000;
        constexpr uint16_t ACK_REQUEST_INTERVAL = 17;

        constexpr uint16_t TRANSMIT_INTERVAL = 500;
        constexpr uint16_t AUX_TIMEOUT_MS = 300;
        constexpr uint16_t RECEIVE_TIMEOUT_MS = 1000;
        constexpr uint16_t AUX_WAIT_TIMEOUT_MS = 500;
        constexpr uint16_t SEND_INTERVAL_MS = 50; // 20 Гц // или 200,// Transmitter
        constexpr  bool    REQUEST_ACK = true; // или false — зависит от режима        
 
        // 🔧 ОБНОВИТЬ для SX1278  CRC
        //====================================================
        constexpr float FREQUENCY = 433.0f;     // MHz
        constexpr uint8_t SPREADING_FACTOR = 7;   // 7-12 SF7
        constexpr float BANDWIDTH = 125.0f;     // kHz (125 kHz) - ДОПУСТИМОЕ ЗНАЧЕНИЕ ДЛЯ SX1278
        constexpr uint8_t CODING_RATE = 5;      // 5-8 (4/5) // 5 -> 4/5, 6 -> 4/6, etc.
        constexpr uint8_t OUTPUT_POWER = 14;    // dBm (обычно 2-20dBm)  // dBm (max 17 for high power modules)
        constexpr uint8_t PREAMBLE_LEN = 8;     // символов // Number of symbols
        constexpr uint8_t SYNC_WORD = 0x12;     // 0x12 - стандартное, 0x34 - пример

        // --- Параметры работы ---
        // constexpr uint32_t SEND_INTERVAL_MS = 50; // 20 Гц
        constexpr uint32_t ACK_TIMEOUT_MS = 20; // 20 мс (только для передатчика)
        constexpr uint8_t RX_PRINT_INTERVAL = 16; // Печатать каждый N-й пакет
        constexpr uint8_t TX_PRINT_INTERVAL = 10;    // Каждые 10 пакетов
    };  // end namespace Radio


    namespace Timing {   

            // Таймауты и интервалы
        constexpr uint16_t TX_TIMEOUT_MS = 200;
        constexpr uint16_t RX_TIMEOUT_MS = 200;
        constexpr uint16_t ACK_TIMEOUT_MS = 300; // заменим старый ACK_TIMEOUT
        constexpr uint16_t RESEND_DELAY_MS = 100; // заменим старый RESEND_DELAY
        constexpr uint8_t MAX_RETRIES = 2;
        constexpr uint16_t STATS_PRINT_INTERVAL_MS = 3500 ;
 
        // Неблокирующие таймеры
        constexpr uint32_t SYSTEM_TEST_INTERVAL = 100;
        constexpr uint32_t SERVO_TEST_DURATION = 3000;
        constexpr uint32_t MOTOR_TEST_DURATION = 2000;
        constexpr uint32_t STATUS_UPDATE_INTERVAL = 5000;
        
        // Таймауты инициализации
        constexpr uint32_t INIT_TIMEOUT_MS = 5000;
        constexpr uint32_t MODULE_INIT_DELAY = 100;
        constexpr uint32_t PRINT_DATACOMSET_INTERVAL = 17 ; // Интервал печати поступивших в RECEIVER данных
        // LED индикация
        constexpr uint8_t LED_PIN = 2; // Используем LED_STATUS пин
        constexpr uint16_t LED_BLINK_INTERVAL_MS = 100;

            //  namespace Timing {
        constexpr uint32_t PACKET_FRESHNESS_MS = 10000;  // Свежесть пакета
        constexpr uint32_t CONNECTION_TIMEOUT_MS = 5000; // Таймаут соединения
            //  };

    }; // end namespace Timing 

    // ================== НАСТРОЙКИ LEDC ==================
    namespace LEDCConfig {
        // Конфигурация LEDC для сервоприводов и моторов
        constexpr ledc_timer_t timer_num = LEDC_TIMER_0; // Используем таймер 0
        constexpr ledc_mode_t mode = LEDC_LOW_SPEED_MODE; // Используем низкоскоростной режим
        constexpr ledc_timer_bit_t duty_resolution = LEDC_TIMER_12_BIT; // 12-битная разрешающая способность (4096 уровней)
        constexpr uint32_t freq_hz = 50; // Частота ШИМ 50 Гц (стандарт для серво)
        constexpr uint32_t CHANNEL_SETUP_DELAY_MS = 10; // Задержка между настройкой каналов

     
        // Тайминги моторов  
        constexpr uint32_t MOTOR_FREQ = 1000;         // Hz
        constexpr uint32_t MOTOR_RESOLUTION = 8;      // бит
        constexpr uint32_t MOTOR_MIN_DUTY = 0;        // 0%
        constexpr uint32_t MOTOR_MAX_DUTY = 255;      // 100%
        
        // Таймауты инициализации
        constexpr uint32_t INIT_TIMEOUT_MS = 5000;
        //        constexpr uint32_t CHANNEL_SETUP_DELAY_MS = 10;

    }; // END namespace LEDCConfig

    // ✅ СТАЛО (ВЕРНО для SG90/MG90S):
    namespace ServoCalibration {
        constexpr uint32_t SERVO_FREQ = 50;              // 50 Гц - стандарт для серво
        constexpr uint32_t MIN_PULSE_WIDTH_US = 500;     // 🔑 ИЗМЕНЕНО: 1000 → 500 мкс (0°)
        constexpr uint32_t MAX_PULSE_WIDTH_US = 2500;    // 🔑 ИЗМЕНЕНО: 2000 → 2500 мкс (180°)
        constexpr uint32_t NEUTRAL_PULSE_WIDTH_US = 1500;// ✅ 1.5 мс = 90° (нейтраль)
    };// end ServoCalibration
};   // end namespace Config

// ================== НАСТРОЙКИ ТЕСТИРОВАНИЯ ==================
namespace TestConfig {
    constexpr uint32_t TEST_STEP_DELAY = 500;     // мс между шагами теста
    constexpr uint32_t SERVO_TEST_DURATION = 3000; // мс теста сервоприводов
    constexpr uint32_t MOTOR_TEST_DURATION = 2000; // мс теста моторов
    constexpr uint32_t SYSTEM_TEST_INTERVAL = 100; // мс между проверками состояния
}; // END namespace TestConfig 

// ================== КОДЫ ОШИБОК ==================
namespace ErrorCodes {
    constexpr int LEDC_INIT_FAILED = 0x10;
    constexpr int LEDC_CHANNEL_CONFIG_FAILED = 0x11;
    constexpr int LEDC_TIMER_CONFIG_FAILED = 0x12;
    constexpr int SERVO_OUT_OF_RANGE = 0x20;
    constexpr int MOTOR_OUT_OF_RANGE = 0x21;
    constexpr int INVALID_SERVO_NUMBER = 0x22;
    constexpr int INVALID_MOTOR_NUMBER = 0x23;
}; // END  namespace ErrorCodes

// ================== ПРЕДЕЛЫ ВАЛИДАЦИИ ==================
namespace ValidationLimits {
    //constexpr int SERVO_MIN_ANGLE = 0;
    //constexpr int SERVO_MAX_ANGLE = 180;
    constexpr int MOTOR_MIN_SPEED = 0;
    constexpr int MOTOR_MAX_SPEED = 100; // 100%
    //          constexpr uint8_t SERVO_COUNT = 5;
    constexpr uint8_t MOTOR_COUNT = 2;


}; // END  namespace ValidationLimits

// Преамбулы для разных типов пакетов
constexpr uint8_t PACKET_PREAMBLE_1 = 0xAA;
constexpr uint8_t PACKET_PREAMBLE_2 = 0x55;
constexpr uint8_t ACK_PREAMBLE_1 = 0x55;
constexpr uint8_t ACK_PREAMBLE_2 = 0xAA;

constexpr uint8_t TELEMETRY_PREAMBLE_1 = 0x56;
constexpr uint8_t TELEMETRY_PREAMBLE_2 = 0xAB;
constexpr uint8_t BATTERY_PREAMBLE_1 = 0x57;
constexpr uint8_t BATTERY_PREAMBLE_2 = 0xAC;

                        // Битовая маска команд
                        //constexpr uint8_t COM_SETALL_MASK = 0b00000001;


                        // Глобальная переменная отладки — объявляется ОДИН РАЗ
                        //extern ControllerDebug_t СDebug;


#endif // CONFIG_H