
//===========================================================
// 2. Config.h - исправляем и документируем
//==========================================================
//
/**
 * @file Config.h
 * @brief Высокоскоростная неблокирующая LoRa связь с подтверждением приема
 * 
 * АРХИТЕКТУРА:
 * ├── FastLoRaTransmitter - передатчик с минимальной задержкой
 * │   ├── Состояния: IDLE → PREPARING → SENDING → WAITING_ACK → IDLE
 * │   └── Время обработки: < 5 мс
 * │
 * └── FastLoRaReceiver - приемник с быстрым ответом
 *     ├── Состояния: LISTENING → PROCESSING → SENDING_ACK → LISTENING
 *     └── Время обработки: < 3 мс
 * 
 * ОСОБЕННОСТИ:
 * ✓ Неблокирующая обработка на основе millis()
 * ✓ Подтверждение приема (ACK) с CRC8 проверкой
 * ✓ Статистика производительности в реальном времени
 * ✓ Автоматическое восстановление при ошибках
 * ✓ Гибкая система отладки с 4 уровнями детализации
 * 
 * ПРОИЗВОДИТЕЛЬНОСТЬ:
 * - Максимальная частота: 20 Гц (50 мс интервал)
 * - Минимальная задержка: 50 мс стабильно
 * - Потеря пакетов: < 0.1% в тестах
 * 
 * @author Embedded Systems Team
 * @version 34.1.1
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <RadioLib.h>
#include <cstdint>
#include <driver/ledc.h>
#include <SPI.h>
#include <esp_log.h>
//#define LOG_TAG "Receiver"

#pragma once
// Версия системы
#define SYSTEM_VERSION  "4.1.1"
#define BUILD_DATE  "28-03-2026"


// Структура отладки
struct ControllerDebug_t {
  bool recieve;
  bool sender;
  bool ebyte;
  bool mcpwm;
  bool ledc;
  bool setup;
  bool loop;
  bool diod;
  bool SLIDE_POT;
  bool JOYSTICK;
  bool PRINT_INCOME;
  bool cc;
};


// ============================================================================
// КОНСТАНТЫ
// ============================================================================


namespace Config {
    // В E49_Config.h, в namespace Config или отдельно
    extern SPIClass* hspi; // <-- Объявляем

    namespace Pins {
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
        constexpr uint8_t VSPI_SCLK = 18;     // ✅ Стандартный SCLK для VSPI
        constexpr uint8_t VSPI_MISO = 19;     // ✅ Стандартный MISO для VSPI  
        constexpr uint8_t VSPI_MOSI = 23;     // ✅ Стандартный MOSI для VSPI

        // ============================================================================
        //  МОДУЛЬ ПОЛУЧЕНИЯ КОМАНД ДЖОЙСТИКА
        // ============================================================================
        constexpr uint8_t JOYSTICK_X = 35;
        constexpr uint8_t JOYSTICK_Y = 34;
        constexpr uint8_t JOYSTICK_BTN = 16;
        //  МОДУЛЬ ПОЛУЧЕНИЯ КОМАНД ОТ SLIDE_POT (ПРАВЛЕНИЕ ГАЗОМ)
        constexpr uint8_t SLIDE_POT = 25;

        constexpr uint8_t STATUS_LED = 2;
        constexpr uint8_t LED_BUILTIN = 2 ;

        constexpr uint8_t LED_STATUS = 2;

        // ============================================================================
        // 🖥️ I2C OLED ДИСПЛЕЙ (1.30")
        // ============================================================================
        // Пины I2C по умолчанию для ESP32 (можно изменить под вашу схему)
        // Если у вас дисплей подключен к тем же пинам, что и PCA9685 или другие I2C устройства,
        // убедитесь, что номера совпадают с физической схемой.
        constexpr int DISPLAY_SDA_PIN = 21; 
        constexpr int DISPLAY_SCL_PIN = 22;
        
        // Адрес дисплея (обычно 0x3C или 0x3D)
        constexpr uint8_t DISPLAY_I2C_ADDRESS = 0x3C;
        
        // Размеры дисплея (стандарт для 1.30" OLED)
        constexpr int DISPLAY_WIDTH = 128;
        constexpr int DISPLAY_HEIGHT = 64;

    }; // namespace Pins

    namespace Display {
        // Настройки обновления экрана
        constexpr uint32_t UPDATE_INTERVAL_MS = 500; // Обновлять не чаще 200мс
    };

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
        #define LORA_ENABLE_DIO1 1  // 🔑 ИЗМЕНИТЬ: 1 = с DIO1, 0 = только DIO0
        #endif

        // ============================================================================
        // 🔧 АВТОПРОВЕРКА КОНФИГУРАЦИИ (compile-time assert)
        // ============================================================================
            //        #if LORA_ENABLE_DIO1 && (Config::Pins::LORA_DIO1 == RADIOLIB_NC)
            //        #error "LORA_ENABLE_DIO1=1, но LORA_DIO1 не задан в Config.h!"
            //        #endif  
        // 🔧 ИСПРАВЛЕНО: Статическая проверка вместо препроцессорной
        // Препроцессор не поддерживает ::, поэтому используем static_assert
        #if LORA_ENABLE_DIO1
        // Проверка будет выполнена на этапе компиляции, а не препроцессинга
        static_assert(Config::Pins::LORA_DIO1 != RADIOLIB_NC, 
                    "LORA_ENABLE_DIO1=1, но LORA_DIO1 не задан в Config.h!");
        #endif            

        //====================================================
        constexpr float FREQUENCY = 433.0f;
        //constexpr uint32_t BANDWIDTH = 125.0; // ← Правильно        
        constexpr float BANDWIDTH = 125.0f; // kHz (125 kHz) - ДОПУСТИМОЕ ЗНАЧЕНИЕ ДЛЯ SX1278
            //constexpr float BANDWIDTH     = 125.0E3f;  // ← float! ИЛИ 125.0E3 125.0E3
        constexpr uint8_t SPREADING_FACTOR = 7;   // SF7
        constexpr uint8_t CODING_RATE = 5;        // CR 4/5
        constexpr uint8_t OUTPUT_POWER = 14;      // 17 dBm
        constexpr uint8_t PREAMBLE_LEN = 8;       // 8 symbols
        constexpr uint8_t SYNC_WORD = 0x12;
        constexpr  bool    REQUEST_ACK = true; // или false — зависит от режима        

        // Более быстрые настройки для тестирования
    /*     
    //constexpr float FREQUENCY = 433.0;
    //constexpr uint32_t BANDWIDTH = 500000;    // 500 kHz для скорости
    // constexpr uint8_t SPREADING_FACTOR = 7;   // SF7 (самый быстрый)
    // constexpr uint8_t CODING_RATE = 5;        // CR 4/5
    // constexpr uint8_t OUTPUT_POWER = 14;
    // constexpr uint8_t PREAMBLE_LEN = 8;       // 6 symbols
    // constexpr uint8_t SYNC_WORD = 0x12;
    */ 

        // 🔑 ДОБАВИТЬ для отладки:
        #define LORA_CONFIG_CHECK() do { \
            ESP_LOGI("LORA_CONFIG", "=== LoRa НАСТРОЙКИ ==="); \
            ESP_LOGI("LORA_CONFIG", "Частота: %.1f MHz", FREQUENCY); \
            ESP_LOGI("LORA_CONFIG", "BW: %.1f kHz", BANDWIDTH); \
            ESP_LOGI("LORA_CONFIG", "SF: %d", SPREADING_FACTOR); \
            ESP_LOGI("LORA_CONFIG", "CR: 4/%d", CODING_RATE); \
            ESP_LOGI("LORA_CONFIG", "SyncWord: 0x%02X", SYNC_WORD); \
            ESP_LOGI("LORA_CONFIG", "Preamble: %d символов", PREAMBLE_LEN); \
            ESP_LOGI("LORA_CONFIG", "Power: %d dBm", OUTPUT_POWER); \
            ESP_LOGI("LORA_CONFIG", "CRC: ВКЛЮЧЕН"); \
            ESP_LOGI("LORA_CONFIG", "======================"); \
        } while(0)

        constexpr uint16_t SEND_INTERVAL_MS = 100; // 100 мс для теста
        constexpr uint16_t TX_TIMEOUT_MS = 100;
        constexpr uint16_t RX_TIMEOUT_MS = 100;
        constexpr uint16_t ACK_TIMEOUT_MS = 150;   // 50 мс для ACKАСКС
        constexpr uint8_t MAX_RETRIES = 1;
        
        constexpr uint8_t TX_PRINT_INTERVAL = 15;   // Каждые 15 пакетов (~1.5 сек при 100 мс)
        constexpr uint8_t RX_PRINT_INTERVAL = 13;   // Каждые 13 пакетов (~1.3 сек при 100 мс)
        constexpr uint32_t ACK_REQUEST_INTERVAL = 1500; // Запрос ACK каждые 1.5 сек

    }; // namespace Radio
    
    namespace Timing {
        constexpr uint16_t INPUT_READ_INTERVAL_MS = 100;
        constexpr uint16_t STATS_PRINT_INTERVAL_MS = 5000;
        constexpr uint16_t LED_BLINK_INTERVAL_MS = 100;
        constexpr uint16_t CONNECTION_TIMEOUT_MS = 5000;
        constexpr uint16_t PACKET_FRESHNESS_MS = 200;
        constexpr uint16_t COLLISION_AVOIDANCE_MS = 50;
        
        constexpr uint16_t INTERVAL_TX_MS = 120 ;
        //constexpr uint32_t SENDER_PRINT_INTERVAL = 2500; // 2.5 сек
        constexpr uint32_t SENDER_PRINT_INTERVAL = 2500; // 2.5 сек
        constexpr uint32_t RECEIVER_PRINT_INTERVAL = 2300; // 2.3 сек
    }; // END namespace Timing


    namespace Debug {
        constexpr bool ENABLE_PACKET_LOGGING = true;
        constexpr uint16_t PACKET_LOG_INTERVAL = 1;
        constexpr bool VALIDATE_CRC = true;
        constexpr bool LOG_RADIO_STATE = true;
    }; // END namespace Debug


  namespace LEDCConfig {
    constexpr uint32_t SERVO_FREQ = 50;
    constexpr uint32_t SERVO_RESOLUTION = 16;
    constexpr uint32_t SERVO_MIN_PULSE = 500;
    constexpr uint32_t SERVO_MAX_PULSE = 2500;
    constexpr uint32_t MOTOR_FREQ = 1000;
    constexpr uint32_t MOTOR_RESOLUTION = 8;

  };// END namespace LEDCConfig
}; // END namespace Config


// Преамбулы
constexpr uint8_t PACKET_PREAMBLE_1 = 0xAA;
constexpr uint8_t PACKET_PREAMBLE_2 = 0x55;
constexpr uint8_t ACK_PREAMBLE_1 = 0x55;
constexpr uint8_t ACK_PREAMBLE_2 = 0xAA;


// Флаги
#define ACK_REQUEST_FLAG   0b00000001


#endif
//==========================================================
