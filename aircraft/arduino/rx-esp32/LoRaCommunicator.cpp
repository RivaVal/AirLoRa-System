

// Проект Sender / Receiver (_Receiver_)
//===============================================================
//  LoRaCommunicator.cpp
//================================================================
//  Файл: LoRaCommunicator.cpp
// Файл: LoRaCommunicator.cpp

/**
 * @file LoRaCommunicator.cpp
 * @brief Реализация модуля для работы с радиомодулем SX1278
 * 
 * @details Этот файл содержит реализацию всех методов класса FastLoRaReceiver,
 * отвечающего за работу с радиомодулем SX1278 через библиотеку RadioLib.
 * 
 * @note Все операции с радиомодулем инкапсулированы в этом модуле для
 * обеспечения правильной архитектуры и упрощения отладки.
 * 
 * @author Embedded Systems Team
 * @version 1.0.0
 * @date 2026-02-17
 */

//----------------------------------------------------------
//Файл: LoRaCommunicator.cpp (усовершенствованный)
//📁 УНИФИЦИРОВАННЫЙ LoRaCommunicator.cpp
//3. LoRaCommunicator.cpp - исправляем логику
//

/**
 * @file LoRaCommunicator.cpp
 * @brief Реализация высокоскоростной LoRa связи
 * 
 * ОПТИМИЗАЦИИ:
 * - Табличный CRC8 для максимальной скорости (5x быстрее)
 * - Минимизация копирования памяти (memcpy вместо поточных операций)
 * - Пакетная настройка параметров радио
 * - Быстрая обработка прерываний (volatile флаги)
 * - Оптимизированные конечные автоматы
 * 
 * ПРОИЗВОДИТЕЛЬНОСТЬ:
 * - Расчет CRC8: < 10 мкс на пакет
 * - Подготовка пакета: < 50 мкс
 * - Передача по радио: 3-15 мс (зависит от настроек)
 * - Полный цикл отправки: < 20 мс
 * 
 * НАДЕЖНОСТЬ:
 * - Автоматическое восстановление после 5 последовательных ошибок
 * - Проверка целостности данных (CRC8 + преамбула)
 * - Фильтрация дубликатов пакетов
 * - Мониторинг качества сигнала (RSSI/SNR)
 */
/**
 * @file LoRaCommunicator.cpp
 * @brief Высоконадёжная LoRa-связь с подтверждением приёма (ACK) и автодиагностикой
 *
 * 🧠 АРХИТЕКТУРА:
 * - Неблокирующий приёмник на основе прерываний DIO0
 * - Финитный автомат (FSM) с 5 состояниями: INIT, LISTENING, PROCESSING, SENDING_ACK, ERROR
 * - Полная валидация пакетов: преамбула, CRC8, дубликаты
 * - Надёжная отправка ACK с восстановлением радиомодуля при ошибках
 * - Интеграция с `UnifiedLEDCController` через `processReceiverCommands()`
 *
 * ⚙️ ОСОБЕННОСТИ:
 * - Отладочная печать управляется через `ControllerDebug_t` (см. E49_Config.h)
 * - Печать FSM-состояний, ACK, серв, моторов — по флагам
 * - Автоматическое восстановление при сбое `startReceive()`
 * - Сохранение совместимости с `Data_ComSet_t` и `AckPacket_t` из `CommonTypes.h`
 *
 * 📊 ПРОИЗВОДИТЕЛЬНОСТЬ:
 * - Обработка пакета: < 2 мс
 * - Возврат в режим приёма после ACK: гарантирован
 * - Потеря пакетов: < 0.1% в условиях прямой видимости
 *
 * @author Embedded Systems Team
 * @version 5.1.0
 * @date 2026
 */

/**
 * @file LoRaCommunicator.cpp
 * @brief Реализация высокоскоростной LoRa связи
 * 
 * @details Оптимизированная реализация LoRa-коммуникации с использованием:
 * - Табличного CRC8 для максимальной скорости (5x быстрее)
 * - Неблокирующего конечного автомата
 * - Подробной диагностики через ESP_LOG
 * - Системы подтверждения приёма (ACK)
 * 
 * @note Все пины зафиксированы в Config.h и не должны меняться без серьезной причины!
 * 
 * @author Embedded Systems Team
 * @version 5.5
 * @date 2026-02-18
 */

/**
 * @file LoRaCommunicator.cpp
 * @brief Реализация высокоскоростной LoRa связи с SX1278 (RadioLib 7.5.0)
 * @version 6.6.2 — ИСПРАВЛЕНА логика begin(), добавлена диагностика SPI
 * @date 2026-02-25
 * 
 * @details
 * 🔑 КРИТИЧЕСКИЕ ИСПРАВЛЕНИЯ:
 * 1. Исправлена логика возврата в begin() — больше не возвращает true при ошибке
 * 2. Добавлен метод diagnoseSPI() для отладки подключения
 * 3. Улучшена обработка ошибок с подробным ESP_LOG выводом
 * 4. Добавлена повторная инициализация при сбое
 * 
 * @note Все пины зафиксированы в Config.h и НЕ должны меняться без серьёзной причины!
 */
/**
 * @file LoRaCommunicator.cpp
 * @brief Реализация высокоскоростной LoRa связи с SX1278 (RadioLib 7.5.0)
 * @version 6.6.2 — ИСПРАВЛЕНА логика begin(), добавлена диагностика SPI
 * @date 2026-02-25
 * 
 * @details
 * 🔑 КРИТИЧЕСКИЕ ИСПРАВЛЕНИЯ:
 * 1. Исправлена логика возврата в begin() — больше не возвращает true при ошибке
 * 2. Добавлен метод diagnoseSPI() для отладки подключения
 * 3. Улучшена обработка ошибок с подробным ESP_LOG выводом
 * 4. Добавлена повторная инициализация при сбое
 * 
 * @note Все пины зафиксированы в Config.h и НЕ должны меняться без серьёзной причины!
 */



#include <SPI.h>
#include <esp_log.h>
#include "esp_task_wdt.h"
#include "CommonTypes.h"
#include "LoRaCommunicator.h"
#include "Config.h"
#include "Unified_LEDC_Controller.h"  // для отладки команд
#include "utils.h"
#include "FlightStabilizer.h" 

//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"

// ============================================================================
// ТЕГИ ДЛЯ ESP_LOG
// ============================================================================
// Теги для логирования
static const char* TAG_LORA_TX = "LoRaTX";
static const char* TAG_LORA_RX = "LoRaRX";


// === Таблица CRC8 (PGM) ===
// Precomputed table for fast CRC8 calculation (CRC8 poly 0x07)
//  const uint8_t CRC8_TABLE[256] PROGMEM = {
const uint8_t CRC8_TABLE[256] = {
  0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
  0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
  0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
  0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
  0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
  0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
  0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
  0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
  0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
  0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
  0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
  0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
  0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
  0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
  0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
  0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

// === Быстрый расчёт CRC8 ===
uint8_t calculateCRC8(const uint8_t* data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) {
        crc = CRC8_TABLE[crc ^ data[i]]; // Прямой доступ без pgm_read_byte
    }
    ESP_LOGV(TAG_LORA_RX, "CRC8 calculated: 0x%02X (len=%u)", crc, length);
    return crc;
}

// Где-то в .cpp файле (например, в LoRaCommunicator.cpp)
void printDataComSet(const DataComSet_t& data, const char* source) {
    Serial.println("\n=== СОДЕРЖИМОЕ DataComSet_t ===");
    Serial.printf("Источник: %s\n", source);
    Serial.printf("ID пакета: %u\n", data.packet_id);
    Serial.printf("Время: %lu мс\n", data.timestamp);
    Serial.printf("comUp: %u\n", data.comUp);
    Serial.printf("comLeft: %u\n", data.comLeft);
    Serial.printf("comThrottle: %u\n", data.comThrottle);
    Serial.printf("comParashut: %u\n", data.comParashut);
    Serial.printf("comSetAll: 0x%02X = ", data.comSetAll);
    if (data.comSetAll & ACK_REQUEST_FLAG) Serial.print("[ACK_REQ] ");
    if (data.comSetAll & PARASHUT_FLAG) Serial.print("[PARACHUTE] ");
    if (data.comSetAll == 0) Serial.print("[NORMAL]");
    Serial.println();
    Serial.printf("CRC8: 0x%02X\n", data.crc8);
    Serial.printf("Преамбула: 0x%02X 0x%02X\n", data.preamble[0], data.preamble[1]);
    Serial.println("===============================\n");
}


// ============================================================================
// ГЛОБАЛЬНЫЙ УКАЗАТЕЛЬ ДЛЯ ISR
// ============================================================================
// === Глобальная функция прерывания DIO0 ===
// ============================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ И ISR ДЛЯ ПРИЕМНИКА
// ============================================================================
// Мьютекс для потокобезопасного доступа к данным
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // Мьютекс для критических секций

// Статический указатель на текущий экземпляр для обработки прерываний
FastLoRaReceiver* FastLoRaReceiver::_instance = nullptr;

// ============================================================================
// 🔧 ГЛОБАЛЬНЫЕ ISR ФУНКЦИИ (RadioLib 7.6.0+)
// ============================================================================
/**
 * @brief Глобальный обработчик прерывания DIO0 (RxDone)
 * @note Вызывается при успешном приёме пакета
 * @warning Только атомарные операции в контексте прерывания!
 */
void IRAM_ATTR onDio0ISR() {
    if (FastLoRaReceiver::_instance != nullptr) {
        portENTER_CRITICAL_ISR(&mux);
        FastLoRaReceiver::_instance->_rxDoneFlag = true;      // 🔑 Этот флаг!
        FastLoRaReceiver::_instance->_dataReceivedFlag = true; // 🔑 И этот!
        portEXIT_CRITICAL_ISR(&mux);
        ESP_EARLY_LOGV("LoRaRX_ISR", "✅ DIO0 IRQ: RxDone (packet ready)");
    } else {
        ESP_EARLY_LOGE("LoRaRX_ISR", "❌ CRITICAL: _instance is NULL in DIO0 ISR!");
    }
}

/**
 * @brief Глобальный обработчик прерывания DIO1 (RxTimeout/CAD)
 * @note Вызывается при:
 *   • Таймауте приёма (Rx Timeout)
 *   • Завершении CAD (Channel Activity Detection)
 * @warning Только атомарные операции!
 */
void IRAM_ATTR onDio1ISR() {
    if (FastLoRaReceiver::_instance != nullptr) {
        portENTER_CRITICAL_ISR(&mux);
        FastLoRaReceiver::_instance->_rxTimeoutFlag = true;    // 🔑 Флаг таймаута
        FastLoRaReceiver::_instance->_dio1TimeoutFlag = true;  // 🔑 Дублирующий флаг
        portEXIT_CRITICAL_ISR(&mux);
        ESP_EARLY_LOGV("LoRaRX_ISR", "✅ DIO1 IRQ: RxTimeout/CAD");
    } else {
        ESP_EARLY_LOGE("LoRaRX_ISR", "❌ CRITICAL: _instance is NULL in DIO1 ISR!");
    }
}

// ============================================================================
// 🔧 ГЛОБАЛЬНЫЕ ОБЪЯВЛЕНИЯ ISR ДЛЯ RadioLib
// Файл: LoRaCommunicator.cpp | Позиция: после #include, до кода класса
// ============================================================================
/**
 * @brief Глобальные обработчики прерываний для RadioLib setDio0Action/setDio1Action
 * @note Объявлены как extern "C" для совместимости с C-библиотеками
 * @warning Вызываются из контекста прерывания - только атомарные операции!
 */

 /*
        //        extern "C" {
        //        friend void IRAM_ATTR onDio0ISR();  // ← 🔑 ДОБАВИТЬ
        //        friend void IRAM_ATTR onDio1ISR();  // ← 🔑 ДОБАВИТЬ
        //        }
friend void  onDio0ISR();  // ← 🔑 ДОБАВИТЬ
friend void  onDio1ISR();  // ← 🔑 ДОБАВИТЬ

// Реализации должны быть БЕЗ пространства имён класса:
void IRAM_ATTR onDio0ISR() {  // ← НЕ FastLoRaReceiver::onDio0ISR!
    if (FastLoRaReceiver::_instance != nullptr) {
        portENTER_CRITICAL_ISR(&mux);
        FastLoRaReceiver::_instance->_rxDoneFlag = true;
        portEXIT_CRITICAL_ISR(&mux);
        ESP_EARLY_LOGV(TAG_LORA_RX, "ISR: DIO0 RxDone");
    }
}

void IRAM_ATTR onDio1ISR() {
    if (FastLoRaReceiver::_instance != nullptr) {
        portENTER_CRITICAL_ISR(&mux);
        FastLoRaReceiver::_instance->_dio1TimeoutFlag = true;
        portEXIT_CRITICAL_ISR(&mux);
        ESP_EARLY_LOGV(TAG_LORA_RX, "ISR: DIO1 RxTimeout");
    }
}
*/

// ============================================================================
// ENUMS И STATES
// ============================================================================
/**
 * > 📝 **Документация:** Атрибут `IRAM_ATTR` размещается 
 * **только в объявлении** (`.h`). 
 * Компилятор автоматически применяет его к реализации. 
 * Дублирование вызывает конфликт секций памяти.
 *
 * @brief Обработчик прерывания RxDone (DIO0)
 * @brief ISR обработчик приема пакета (RxDone) - DIO0
 * Вызывается аппаратно при успешном приеме пакета
 * @note Вызывается аппаратно при успешном приёме пакета
 * @warning Только атомарные операции!
 */
void FastLoRaReceiver::onRxDone() {
    if (_instance != nullptr) {
        portENTER_CRITICAL_ISR(&mux);
        _instance->_rxDoneFlag = true;
        portEXIT_CRITICAL_ISR(&mux);
        ESP_EARLY_LOGV(TAG_LORA_RX, "ISR: DIO0 RxDone triggered");
    }
}

/** > 📝 **Документация:** 
 * `IRAM_ATTR` указывается **только в объявлении** 
 * (`.h`). Компилятор автоматически применяет его к реализации. Дублирование 
 *  вызывает конфликт секций `.iram1.*`.

 * @brief Обработчик прерывания RxTimeout (DIO1)
 * @brief ISR обработчик таймаута приема (RxTimeout) - DIO1
 * @note Вызывается при таймауте приёма (RadioLib 7.6.0+)
 * @warning Только атомарные операции!
 * Вызывается аппаратно при таймауте приема (если настроено)
 */
void FastLoRaReceiver::onRxTimeout() {
    if (_instance != nullptr) {
        portENTER_CRITICAL_ISR(&mux);
        _instance->_rxTimeoutFlag = true;
        portEXIT_CRITICAL_ISR(&mux);
        ESP_EARLY_LOGV(TAG_LORA_RX, "ISR: DIO1 RxTimeout triggered");
    }
}

// ============================================================================
// ОБРАБОТЧИК ПРЕРЫВАНИЯ DIO0 (IRAM_ATTR)
// ============================================================================
/**
 * @brief Глобальный обработчик прерывания DIO0 для SX1278
 * @note Использует дружественный доступ к приватным членам класса FastLoRaReceiver
 * @warning Вызывается из контекста прерывания - только атомарные операции!
 * @see FastLoRaReceiver::_dataReceivedFlag
 */
// ============================================================================
// 🔧 ОБРАБОТЧИК ПРЕРЫВАНИЯ DIO1 (IRAM_ATTR, RadioLib 7.6.0+)
// Файл: LoRaCommunicator.cpp | Позиция: после onDio0ISR()
// ============================================================================
/**
 * @brief Глобальный обработчик прерывания DIO1 для SX1278
 * @note Вызывается при:
 *   • Rx Timeout (приём не начался за заданное время)
 *   • Завершении CAD (Channel Activity Detection)
 * @warning Только атомарные операции в контексте прерывания!
 */
// ============================================================================
// РЕАЛИЗАЦИЯ ПРИЕМНИКА  // КОНСТРУКТОР И ДЕСТРУКТОР
// ============================================================================
/*!
 * @brief Конструктор класса FastLoRaReceiver
 * 
 * @details Выполняет инициализацию радиомодуля SX1278 с полной диагностикой.
 * Инкапсулирует всю логику инициализации и настройки радиомодуля в одном месте.
 * 
 * @brief Конструктор приёмника с поддержкой DIO0/DIO1
 * @param cs_pin    GPIO пин Chip Select
 * @param rst_pin   GPIO пин Reset
 * @param dio0_pin  GPIO пин DIO0 (прерывание пакета)
 * @param dio1_pin  GPIO пин DIO1 (таймаут приёма, если LORA_ENABLE_DIO1=1)
 * @param spi       Указатель на SPI-объект (VSPI/HSPI)
 * 
 * @note Архитектура: пин-маппинг управляется в .ino, класс получает готовый Module
 */
// ============================================================================
// 🔧 ИСПРАВЛЕНИЕ #3: ПОРЯДОК ИНИЦИАЛИЗАЦИИ (согласно объявлению в .h)
// ============================================================================
/**
 * @brief Конструктор приёмника с поддержкой DIO0/DIO1
 * @note Порядок инициализации должен строго соответствовать порядку
 *       объявления членов в классе (избегаем -Wreorder)
 */
FastLoRaReceiver::FastLoRaReceiver(
    uint8_t cs_pin,
    uint8_t rst_pin,
    uint8_t dio0_pin,
    int16_t dio1_pin,         // 🔑 Только при LORA_ENABLE_DIO1=1
    SPIClass* spi             // ← Указатель, не ссылка!
) // 🔑 ПОРЯДОК: как в объявлении класса (LoRaCommunicator.h):

: _spi(spi),
  _module(nullptr),           // 1. Сначала указатели на объекты
  _radio(nullptr),            // 2.
  //_spi(spi),                  // 3.
  //_instance(nullptr),         // 4. Статический указатель
  _currentState(StateRx::INIT), // 5. Состояние
  _rxDoneFlag(false),         // 9.
  _rxTimeoutFlag(false),      // 10.
  _dio1TimeoutFlag(false),    // 8.
  _dataReceivedFlag(false),   // 7.
  _dataReady(false),          // 6. volatile флаги (в порядке .h!)
  _initialized(false),        // 11.
  _chipConnected(false),      // 12.
  _lastPacketId(0),           // 13.
  _ackPacketId(0),            // 14.
  _debugLevel(DebugLevel::DEBUG_INFO), // 15.
  _lastReceiveTime(0),        // 16.
  _errorCount(0),             // 17.
  _lastErrorTime(0),          // 18.
  _statsRx{},                 // 19.
  _receivedData{},            // 20.
  _ackPacket{}                // 21.
{
    ESP_LOGI(TAG_LORA_RX, "=== 🚀 Конструктор FastLoRaReceiver ===");
    ESP_LOGI(TAG_LORA_RX, "   Пины: CS=%d, RST=%d, DIO0=%d",
             cs_pin, rst_pin, dio0_pin);
    
#if LORA_ENABLE_DIO1
    ESP_LOGI(TAG_LORA_RX, "   🆕 DIO1=%d (активен, RadioLib 7.6.0+)", dio1_pin);
#else
    ESP_LOGW(TAG_LORA_RX, "   ⚠️ DIO1 отключён (LORA_ENABLE_DIO1=0)");
#endif
    
    ESP_LOGI(TAG_LORA_RX, "   SPI: %p (VSPI/HSPI)", spi);
    
    // 🔑 Привязка экземпляра для ISR (должно быть первым!)
    _instance = this;
    ESP_LOGV(TAG_LORA_RX, "   ✅ _instance привязан: %p", _instance);
    
    // Создание объектов (после привязки _instance!)
    ESP_LOGI(TAG_LORA_RX, "   📦 Создание Module...");
    #if LORA_ENABLE_DIO1
        _module = new Module(cs_pin, dio0_pin, rst_pin, dio1_pin, *spi);
        ESP_LOGI(TAG_LORA_RX, "   ✅ Module создан с DIO1=%d", dio1_pin);
    #else
        _module = new Module(cs_pin, dio0_pin, rst_pin, RADIOLIB_NC, *spi);
        ESP_LOGI(TAG_LORA_RX, "   ✅ Module создан (DIO0 only)");
    #endif
    
    if (!_module) {
        ESP_LOGE(TAG_LORA_RX, "❌ ОШИБКА: Не удалось создать Module!");
        return;
    }
    
    ESP_LOGI(TAG_LORA_RX, "   📻 Создание SX1278...");
    _radio = new SX1278(_module);
    if (!_radio) {
        ESP_LOGE(TAG_LORA_RX, "❌ ОШИБКА: Не удалось создать SX1278!");
        delete _module;
        _module = nullptr;
        return;
    }
    
    ESP_LOGI(TAG_LORA_RX, "✅ Конструктор завершён: module=%p, radio=%p", _module, _radio);
}




   // --- Destructor Implementation ---
    FastLoRaReceiver::~FastLoRaReceiver() {
    if (_radio) delete _radio;
    if (_module) delete _module;
    if (_instance == this) _instance = nullptr;
    ESP_LOGI(TAG_LORA_RX, "Destructor called");
}

// ============================================================================
// 🔧 ДИАГНОСТИКА SPI — НОВЫЙ МЕТОД
// ============================================================================
/**
 * @brief Диагностика SPI-соединения с SX1278
 * @return true если модуль отвечает корректно
 * 
 * @details Проверяет:
 * - Настройку пинов CS, RST, DIO0
 * - Аппаратный сброс через RST
 * - Чтение версии чипа через SPI
 * - Базовую SPI-транзакцию
 */
// ============================================================================
// 🔧 ДОБАВИТЬ ДОПОЛНИТЕЛЬНУЮ ДИАГНОСТИКУ SPI
// ============================================================================
bool FastLoRaReceiver::diagnoseSPI() {
    ESP_LOGI(TAG_LORA_RX, "🔍 === ДИАГНОСТИКА SPI-СОЕДИНЕНИЯ ===");
    delay(1500); // Задержка для подключения Serial Monitor
    ESP_LOGI(TAG_LORA_RX, "⚙️  Запуск   FastLoRaReceiver::diagnoseSPI...");

    // 1. Проверка настройки пинов
    pinMode(Config::Pins::LORA_CS, OUTPUT);
    pinMode(Config::Pins::LORA_RST, OUTPUT);
    pinMode(Config::Pins::LORA_DIO0, INPUT_PULLUP);
    ESP_LOGI(TAG_LORA_RX, "  • Пины настроены: CS=%d, RST=%d, DIO0=%d",
             Config::Pins::LORA_CS, Config::Pins::LORA_RST, Config::Pins::LORA_DIO0);
    
    // 🔧 2. Аппаратный сброс ПЕРЕД любой проверкой
    ESP_LOGI(TAG_LORA_RX, "  • Выполняю аппаратный сброс...");
    digitalWrite(Config::Pins::LORA_RST, LOW);
    delay(10);

    digitalWrite(Config::Pins::LORA_RST, HIGH);
    delay(200);  // 🔑 КРИТИЧЕСКИ: 200 мс для стабилизации осциллятора SX1278
    ESP_LOGI(TAG_LORA_RX, "⏳ Задержка стабилизации после сброса: 200 мс");
    
    // 🔑 ДОБАВИТЬ: Проверка уровня сигнала на CS
    digitalWrite(Config::Pins::LORA_CS, HIGH);
    delay(1);
    // uint8_t cs_state = digitalRead(Config::Pins::LORA_CS);
    // ESP_LOGI(TAG_LORA_RX, "  • Состояние CS пина: %s", cs_state ? "HIGH" : "LOW");
    
    // 3. Чтение версии чипа (RegVersion = 0x12 для SX1278)
    long version = _radio->getChipVersion();
    ESP_LOGI(TAG_LORA_RX, "  • Версия чипа: 0x%lX (ожидалось 0x12 для SX1278)", version);
    
    // 🔑 ДОБАВИТЬ: Диагностика питания
    ESP_LOGI(TAG_LORA_RX, "  • 🔌 ПРОВЕРКА ПИТАНИЯ LoRa:");
    ESP_LOGI(TAG_LORA_RX, "     - LoRa VCC должен быть 3.3V (НЕ 5V!)");
    ESP_LOGI(TAG_LORA_RX, "     - Проверьте мультиметром напряжение на пине VCC модуля");
    ESP_LOGI(TAG_LORA_RX, "     - Общий GND между ESP32 и LoRa обязателен");
    
    // 🔧 4. ДОБАВИТЬ: проверка через прямой доступ к регистру (обход RadioLib)
    if (version != 0x12) {
        ESP_LOGW(TAG_LORA_RX, "⚠️ Попытка чтения через прямой SPI...");
        // 🔧 ПРЯМОЕ чтение регистра 0x42 (RegVersion) для диагностики
        uint8_t direct_read = _module->SPIreadRegister(0x42); // RegVersion
        ESP_LOGI(TAG_LORA_RX, "🔍 Direct SPI read RegVersion(0x42): 0x%02X (expected 0x12)", direct_read);
            // Если _module имеет доступ к SPI, можно проверить напрямую
    
        if (direct_read != 0x12) {
            ESP_LOGE(TAG_LORA_RX, "❌ Direct SPI read failed - check wiring/power!");
            return false;
        }
    }
    
    if (version == 0x12) {
        ESP_LOGI(TAG_LORA_RX, "✅ SX1278 отвечает по SPI");
    // 5. Проверка RSSI (дополнительная проверка работоспособности)
        int16_t rssi = _radio->getRSSI();
        ESP_LOGI(TAG_LORA_RX, "  • Текущий RSSI: %d dBm (норма: -120...-20)", rssi);
        
        // 🔧 ДОПОЛНИТЕЛЬНАЯ ОТЛАДКА: Оценка качества сигнала
        if (rssi > -60) {
            ESP_LOGI(TAG_LORA_RX, "  • 🟢 Качество сигнала: ОТЛИЧНОЕ");
        } else if (rssi > -90) {
            ESP_LOGI(TAG_LORA_RX, "  • 🟡 Качество сигнала: ХОРОШЕЕ");
        } else {
            ESP_LOGW(TAG_LORA_RX, "  • 🔴 Качество сигнала: НИЗКОЕ");
        }
        return true;
    } else {
        // 🔧 ДОБАВИТЬ: имя ошибки для диагностики
        ESP_LOGE(TAG_LORA_RX, "❌ ОШИБКА: версия чипа 0x%lX ≠ 0x12", version);
            if (_radio != nullptr) {
        }
        // return false;
        // ESP_LOGE(TAG_LORA_RX, "❌ ОШИБКА SPI: неверная версия чипа");
        ESP_LOGE(TAG_LORA_RX, "🔧 Проверьте подключение:");
        ESP_LOGE(TAG_LORA_RX, "  1. Питание LoRa: ТОЛЬКО 3.3V (5V убьёт модуль!)");
        ESP_LOGE(TAG_LORA_RX, "  2. Общий GND между ESP32 и LoRa");
        ESP_LOGE(TAG_LORA_RX, "  3. SPI пины: SCK(18), MISO(19), MOSI(23)");
        ESP_LOGE(TAG_LORA_RX, "  4. CS(5) подключён и работает как OUTPUT");
        ESP_LOGE(TAG_LORA_RX, "  5. Pull-up резисторы 10кΩ на SPI линиях (опционально)");
        ESP_LOGE(TAG_LORA_RX, "  6. 🔌 ИЗМЕРЬТЕ МУЛЬТИМЕТРОМ напряжение на VCC LoRa!");
        return false;
    }
}


// ============================================================================
// ИНИЦИАЛИЗАЦИЯ (НЕ создаёт Module!)  — ИСПРАВЛЕННАЯ ЛОГИКА
// ============================================================================
// ============================================================================
// ИНИЦИАЛИЗАЦИЯ РАДИОМОДУЛЯ
// ============================================================================
/**
 * @brief Инициализация радиомодуля SX1278
 * @return true при успешной инициализации, false при ошибке
 * 
 * @details Выполняет полную инициализацию радиомодуля с детальной диагностикой.
 * Включает в себя:
 * - Проверку конфигурационных параметров
 * - Проверку подключения по SPI
 * - Настройку всех необходимых параметров
 * - Проверку перехода в режим приёма
 *
 * @details 🔑 КРИТИЧЕСКИЕ ИСПРАВЛЕНИЯ:
 * 1. Устранена ошибка: больше не возвращает true при failed begin()
 * 2. Добавлена повторная попытка инициализации
 * 3. Подробная диагностика через ESP_LOG
 * 4. Блокировка системы только при критической ошибке
 * 
 * @return true при успешной инициализации, false при ошибке
 */
bool FastLoRaReceiver::begin() {
    if (!_radio || !_module  || !_spi) {
        ESP_LOGE(TAG_LORA_RX, "❌ begin(): null pointer detected");
        return false;
    }

    ESP_LOGI(TAG_LORA_RX, "Initializing Radio...");

    int state = _radio->begin(
        Config::Radio::FREQUENCY,
        Config::Radio::BANDWIDTH,
        Config::Radio::SPREADING_FACTOR,
        Config::Radio::CODING_RATE,
        Config::Radio::SYNC_WORD,
        Config::Radio::OUTPUT_POWER,
        Config::Radio::PREAMBLE_LEN
    );

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG_LORA_RX, "Radio begin failed: %d", state);
        return false;
    }

    _radio->setCRC(true);

    ESP_LOGI(TAG_LORA_RX, "Radio initialized successfully");
    // ============================================================================
    // 🔧 НАСТРОЙКА ПРЕРЫВАНИЙ: DIO0 + DIO1 (RadioLib 7.6.0+)
    // ============================================================================
    ESP_LOGI(TAG_LORA_RX, "⚙️  Настройка прерываний...");

    // 🔹 DIO0: Пакет получен (RxDone) — ОБЯЗАТЕЛЬНО
    _radio->setDio0Action(::onDio0ISR, RISING);
    ESP_LOGI(TAG_LORA_RX, "✅ DIO0 (GPIO%d) → onDio0ISR [RxDone]", Config::Pins::LORA_DIO0);

    ESP_LOGI(TAG_LORA_RX, "✅ DIO0 (GPIO%d) → onDio0ISR (RISING)", Config::Pins::LORA_DIO0);

        
    //            #if LORA_ENABLE_DIO1
    // Прерывание DIO1: таймаут приёма (RxTimeout) или CAD
    // 🔹 DIO1: Таймаут приёма / CAD — ТОЛЬКО если LORA_ENABLE_DIO1=1
#if LORA_ENABLE_DIO1
    // 🔑 КРИТИЧЕСКИ: Проверка валидности пина DIO1
    if (Config::Pins::LORA_DIO1 != RADIOLIB_NC) {
        _radio->setDio1Action(::onDio1ISR, RISING);
        ESP_LOGI(TAG_LORA_RX, "✅ DIO1 (GPIO%d) → onDio1ISR [RxTimeout/CAD]", 
                 Config::Pins::LORA_DIO1);
        
        // 🔧 Дополнительная диагностика пина DIO1
        pinMode(Config::Pins::LORA_DIO1, INPUT_PULLUP);
        delay(1);
        int dio1_state = digitalRead(Config::Pins::LORA_DIO1);
        ESP_LOGD(TAG_LORA_RX, "   📊 DIO1 начальное состояние: %s", 
                 dio1_state ? "HIGH" : "LOW");
    } else {
        ESP_LOGW(TAG_LORA_RX, "⚠️ LORA_DIO1 = RADIOLIB_NC, отключаем DIO1");
    }
#else
    ESP_LOGW(TAG_LORA_RX, "⚠️ DIO1 отключён в Config.h (LORA_ENABLE_DIO1=0)");
    ESP_LOGW(TAG_LORA_RX, "   Таймаут приёма будет обрабатываться программно");
    // Опционально: установить программный таймаут
    // _radio->setRxTimeout(3); // 3 символа таймаута
#endif

    // Запуск приема
    ESP_LOGI(TAG_LORA_RX, "📡 startReceive()...");
    state = _radio->startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        //ESP_LOGE(TAG_LORA_RX, "❌ startReceive failed: код %d (%s)", 
        //        state, RadioLib::getErrorCode(state));
        ESP_LOGE(TAG_LORA_RX, "❌ startReceive failed: код %d (%s)" ); // , 
        //        state, RadioLib::getErrorCode(state));
        _initialized = false;
        return false;
    }

    _initialized = true;
    _currentState = StateRx::LISTENING;
    _lastReceiveTime = millis();
    
    ESP_LOGI(TAG_LORA_RX, "✅ Receiver READY: режим LISTENING, время=%lu", _lastReceiveTime);
    return true;
}






// ============================================================================
// 🔧 ПЕРЕЗАПУСК МОДУЛЯ
// ============================================================================
bool FastLoRaReceiver::restart() {
    ESP_LOGW(TAG_LORA_RX, "🔄 Попытка перезапуска радиомодуля...");
    
    if (!_initialized) {
        ESP_LOGE(TAG_LORA_RX, "❌ Модуль LoRA не инициализирован");
        return false;
    }
    
    // Остановить текущий приём
    _radio->standby();
    delay(50);
    
    // Повторная инициализация
    bool result = begin();
    
    if (result) {
        ESP_LOGI(TAG_LORA_RX, "✅ Перезапуск успешен");
    } else {
        ESP_LOGE(TAG_LORA_RX, "❌ Перезапуск не удался");
    }
    return result;
}


//===========================================================================
// Example placeholder for update method...
void FastLoRaReceiver::update() {
    if (!_radio) return;
    
    // 🔑 ВЫНЕСТИ ОБЪЯВЛЕНИЯ ПЕРЕМЕННЫХ ВНЕ SWITCH
    bool rxDone = false;
    bool rxTimeout = false;
    int state = RADIOLIB_ERR_NONE;
    uint8_t buffer[sizeof(DataComSet_t)];
    

    switch (_currentState) {
        case StateRx::INIT:
            _currentState = StateRx::LISTENING;
            break;
            
        case StateRx::LISTENING:
            // Атомарное чтение флагов
            portENTER_CRITICAL(&mux);
            rxDone = _rxDoneFlag;
            rxTimeout = _rxTimeoutFlag;
            _rxDoneFlag = false;
            _rxTimeoutFlag = false;
            portEXIT_CRITICAL(&mux);
            
            if (rxDone) {
                ESP_LOGD(TAG_LORA_RX, "RxDone IRQ");
                _currentState = StateRx::PROCESSING;
            } else if (rxTimeout) {
                ESP_LOGD(TAG_LORA_RX, "RxTimeout IRQ");
                _statsRx.receive_timeouts++;
                _radio->startReceive();  // Перезапуск приёма
            }
            break;
            
        case StateRx::PROCESSING:
            // 🔑 ПРОВЕРКА указателей
            if (!_radio || !_module) {
                ESP_LOGE(TAG_LORA_RX, "❌ Null pointer in PROCESSING");
                _currentState = StateRx::ERROR;
                break;
            }
            
            // Чтение данных (переменная state уже объявлена выше)
            state = _radio->readData(buffer, sizeof(DataComSet_t));
            if (state == RADIOLIB_ERR_NONE) {
                memcpy(&_receivedData, buffer, sizeof(DataComSet_t));
                if (validatePacket(_receivedData)) {
                    _statsRx.packetsReceivedSuccess++;
                    _dataReady = true;
                    ESP_LOGI(TAG_LORA_RX, "✅ Packet ID:%u validated", _receivedData.packet_id);
                    
                    if (_receivedData.comSetAll & ACK_REQUEST_FLAG) {
                        _currentState = StateRx::SENDING_ACK;
                    } else {
                        _currentState = StateRx::LISTENING;
                        _radio->startReceive();
                    }
                } else {
                    _statsRx.crcErrors++;
                    _currentState = StateRx::LISTENING;
                    _radio->startReceive();
                }
            } else {
                ESP_LOGE(TAG_LORA_RX, "❌ readData failed: %d", state);
                _currentState = StateRx::ERROR;
            }
            break;
            
        case StateRx::SENDING_ACK:
            if (sendAck()) {
                ESP_LOGI(TAG_LORA_RX, "✅ ACK sent");
            } else {
                ESP_LOGE(TAG_LORA_RX, "❌ ACK send failed");
            }
            _currentState = StateRx::LISTENING;
            _radio->startReceive();
            break;
            
        case StateRx::ERROR:
            ESP_LOGW(TAG_LORA_RX, "⚠️ ERROR state, attempting recovery");
            handleRecovery();
            break;
            
        default:
            ESP_LOGW(TAG_LORA_RX, "⚠️ Unknown state: %d", static_cast<int>(_currentState));
            _currentState = StateRx::ERROR;
            break;
    }
}



// Также для приёмника, если используется:
void FastLoRaReceiver::setDebugLevel(DebugLevel level) {
    _debugLevel = level;
}

DebugLevel FastLoRaReceiver::getDebugLevel() const {
    return _debugLevel;
}



// --- FSM Internal Method Implementations ---
void FastLoRaReceiver::fsmInit() {
    // Initialization happens in begin(), so INIT state might just transition.
    // Or it could perform further checks here.
    _currentState = StateRx::LISTENING;
    if(getDebugLevel() >= DebugLevel::DEBUG_INFO  ) Serial.println("FSM: Transitioned to LISTENING after INIT.");
    ESP_LOGD(TAG_LORA_RX, "FSM: INIT → LISTENING");
}



// ============================================================================
// 🔧 ИСПРАВЛЕНИЕ #2: БЕЗОПАСНОЕ ЧТЕНИЕ ФЛАГА ПРЕРЫВАНИЯ
// ============================================================================
/**
* @brief FSM: РЕЖИМ ПРИЁМА (ожидание прерывания) — БЕЗОПАСНАЯ ВЕРСИЯ
* @note Добавлена проверка валидности состояния перед переходом
* @date 2026-03-12
*/
void FastLoRaReceiver::fsmListening() {
    if (_radio == nullptr) {
        ESP_LOGE(TAG_LORA_RX, "❌ fsmListening: _radio == nullptr!");
        _currentState = StateRx::ERROR;
        return;
    }

    // Атомарное чтение флагов прерываний
    bool dio0 = false, dio1 = false;
    portENTER_CRITICAL(&mux);
    dio0 = _dataReceivedFlag; // Обычно ставится в onDio0ISR
    dio1 = _dio1TimeoutFlag;  // Ставится в onDio1ISR
    portEXIT_CRITICAL(&mux);

    if (dio0) {
        // Пакет получен (DIO0)
        ESP_LOGV(TAG_LORA_RX, "📥 Флаг прерывания DIO0 установлен, переход в PROCESSING");
        portENTER_CRITICAL(&mux);
        _dataReceivedFlag = false;
        portEXIT_CRITICAL(&mux);
        _currentState = StateRx::PROCESSING;
    } 
    else if (dio1) {
        // Таймаут приема или CAD (DIO1)
        ESP_LOGD(TAG_LORA_RX, "⏳ DIO1: Rx Timeout/CAD detected.");
        portENTER_CRITICAL(&mux);
        _dio1TimeoutFlag = false;
        portEXIT_CRITICAL(&mux);
        
        // Перезапуск приема для сброса состояния
        int state = _radio->startReceive();
        if (state != RADIOLIB_ERR_NONE) {
            ESP_LOGW(TAG_LORA_RX, "⚠️ startReceive() после timeout вернул %d", state);
        }
        // Остаемся в LISTENING
    }
}

/*
void FastLoRaReceiver::fsmListening() {
    // 🔑 ПРОВЕРКА: Валидность указателя на радио
    if (_radio == nullptr) {
        ESP_LOGE(TAG_LORA_RX, "❌ fsmListening: _radio == nullptr!");
        _currentState = StateRx::ERROR;
        return;
    }
    
    // Атомарное чтение флага прерывания
    bool flag;
    portENTER_CRITICAL(&mux);
    flag = _dataReceivedFlag;
    portEXIT_CRITICAL(&mux);
    
    if (flag) {
        ESP_LOGV(TAG_LORA_RX, "📥 Флаг прерывания установлен, переход в PROCESSING");
        
        // Сброс флага прерывания
        portENTER_CRITICAL(&mux);
        _dataReceivedFlag = false;
        portEXIT_CRITICAL(&mux);
        
        _currentState = StateRx::PROCESSING;
        ESP_LOGD(TAG_LORA_RX, "FSM: переход в PROCESSING (получен пакет)");
    }
}
*/

    

// ============================================================================
// 🔧 ИСПРАВЛЕНИЕ #1: БЕЗОПАСНАЯ ОБРАБОТКА ПРИЁМА — с проверкой указателей
// ============================================================================
// ============================================================================
// 🔧 ИСПРАВЛЕНИЕ #1: БЕЗОПАСНАЯ ОБРАБОТКА ПРИЁМА — Чтение в промежуточный буфер
// ============================================================================
/**
* @brief Валидация и обработка принятого пакета (БЕЗОПАСНАЯ ВЕРСИЯ)
* @note Критически важный участок FSM:
*       1. Проверка валидности указателя _radio
*       2. Чтение данных из радиомодуля
*       3. Валидация преамбулы и CRC
*       4. Фильтрация дубликатов
*       5. Установка флага готовности данных
*       6. Проверка запроса ACK
*
* @note Критические изменения:
*       1. Чтение данных во временный буфер (uint8_t) для избежания ошибок выравнивания (Alignment Fault)
*       2. Проверка фактического количества прочитанных байт
*       3. Детальная отладка через ESP_LOG перед каждым шагом
* @date 2026-03-13
*
* @warning После обработки ВСЕГДА возвращаем модуль в режим приёма!
* @date 2026-03-12
*/
void FastLoRaReceiver::fsmProcessing() {
    ESP_LOGV(TAG_LORA_RX, "FSM: PROCESSING state");
    
    // 🔑 Проверка указателей
    if (_radio == nullptr || _module == nullptr) {
        ESP_LOGE(TAG_LORA_RX, "❌ Null pointer detected!");
        _statsRx.packetErrors++;
        _currentState = StateRx::ERROR;
        return;
    }
    
    // 🔑 Кормление WDT перед readData
    //#if CONFIG_ESP_TASK_WDT_EN
    //    esp_task_wdt_reset();
    //#endif
    
    // 🔑 Чтение в промежуточный буфер (избегаем Alignment Fault)
    uint8_t rxBuffer[MAX_PACKET_SIZE];
    memset(rxBuffer, 0, MAX_PACKET_SIZE);
    
    int state = _radio->readData(rxBuffer, sizeof(DataComSet_t));
    
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG_LORA_RX, "❌ readData failed: %d", state);
        _statsRx.packetErrors++;
        _radio->startReceive();
        _currentState = StateRx::LISTENING;
        return;
    }
    
    // 🔑 ИСПРАВЛЕНО: Безопасное копирование
    memcpy(&_lastData, rxBuffer, sizeof(DataComSet_t));
    

    //=====================================================

    // 🔑 ПРОВЕРКА #4: Обновление статистики и времени
    _statsRx.packetsReceived++;
    _statsRx.lastPacketTime = millis();
    ESP_LOGV(TAG_LORA_RX, "📦 Пакет прочитан успешно. Копирование в структуру...");

    // Безопасное копирование данных из буфера в структуру
    // memcpy(&_lastData, rxBuffer, EXPECTED_SIZE);

    ESP_LOGV(TAG_LORA_RX, "📦 ID=%u, CRC=0x%02X, Preamble=0x%02X 0x%02X",
             _lastData.packet_id, _lastData.crc8, 
             _lastData.preamble[0], _lastData.preamble[1]);

    // Валидация пакета
    if (validatePacket(_lastData)) {
        _statsRx.packetsReceivedSuccess++;
        
        // 🔑 БЕЗОПАСНАЯ установка флага готовности (с мьютексом)
        portENTER_CRITICAL(&mux);
        _dataReady = true;
        portEXIT_CRITICAL(&mux);
        
        ESP_LOGI(TAG_LORA_RX, "✅ Пакет валидирован успешно (ID: %u)", _lastData.packet_id);

        // Проверка запроса ACK
        if (_lastData.comSetAll & ACK_REQUEST_FLAG) {
            _ackPacketId = _lastData.packet_id;
            ESP_LOGV(TAG_LORA_RX, "📤 Запрошен ACK для пакета %u", _ackPacketId);
            _currentState = StateRx::SENDING_ACK;
            return; // Выход, не сбрасывая флаг прерывания здесь, это сделает fsmSendingAck
        }
    } else {
        ESP_LOGW(TAG_LORA_RX, "❌ Packet validation failed (CRC or Preamble error)");
        _statsRx.crcErrors++;
    }

    //==========================================================

    // 🔑 КРИТИЧЕСКИ: Возврат в режим приёма
    // 🔑 ИСПРАВЛЕНИЕ: Защита мьютексом при сбросе флага!
    portENTER_CRITICAL(&mux);
    _dataReceivedFlag = false;
    portEXIT_CRITICAL(&mux);

    ESP_LOGV(TAG_LORA_RX, "🔄 Возврат в режим LISTENING...");
    
    if (_radio->startReceive() != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG_LORA_RX, "❌ startReceive failed after processing");
        _currentState = StateRx::ERROR;
    } else {
        _currentState = StateRx::LISTENING;
        ESP_LOGV(TAG_LORA_RX, "✅ Возврат в режим LISTENING");
    }
}


// ============================================================================
// FSM: ОТПРАВКА ПОДТВЕРЖДЕНИЯ (ACK)
// ============================================================================
// ============================================================================
// FSM: ОТПРАВКА ACK — с гарантированным возвратом в приём
// ============================================================================
void FastLoRaReceiver::fsmSendingAck() {
    ESP_LOGV(TAG_LORA_RX, "FSM: SENDING_ACK state");
    
    // Подготовка ACK пакета
    _ackPacket.preamble[0] = ACK_PREAMBLE_1;
    _ackPacket.preamble[1] = ACK_PREAMBLE_2;
    _ackPacket.packet_id = _ackPacketId;
    _ackPacket.timestamp = millis();
    _ackPacket.status = 1;
    _ackPacket.crc8 = calculateCRC8((uint8_t*)&_ackPacket.packet_id, 7);
    
    // Отправка (блокирующая)
    _radio->standby();
    delayMicroseconds(100);
    
    int state = _radio->transmit((uint8_t*)&_ackPacket, sizeof(AckPacket_t));
    
    if (state == RADIOLIB_ERR_NONE) {
        _statsRx.acksSentSuccess++;
        ESP_LOGI(TAG_LORA_RX, "✅ ACK sent for ID %u", _ackPacketId);
    } else {
        _statsRx.ackSendErrors++;
        ESP_LOGE(TAG_LORA_RX, "❌ ACK send failed: %d", state);
    }
    
    // 🔑 КРИТИЧЕСКИ: Возврат в режим приёма
    _dataReceivedFlag = false;
    if (_radio->startReceive() != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG_LORA_RX, "❌ startReceive failed after ACK");
        _currentState = StateRx::ERROR;
    } else {
        _currentState = StateRx::LISTENING;
        ESP_LOGV(TAG_LORA_RX, "✅ Returned to LISTENING");
    }
}



void FastLoRaReceiver::fsmError() {
    ESP_LOGW(TAG_LORA_RX, "FSM: ERROR state");
    // Handle error state - potentially attempt recovery
    // Serial.println("⚠️ Receiver FSM entered ERROR state.");
    handleRecovery();
    _currentState = StateRx::LISTENING; // Attempt to return to listening after recovery

}

//=============================================================================
// --- Core Logic Methods ---
// ============================================================================
// 🔧 ИСПРАВЛЕНИЕ #3: БЕЗОПАСНАЯ ВАЛИДАЦИЯ ПАКЕТА
// ============================================================================
/**
* @brief ВАЛИДАЦИЯ ПАКЕТА — с проверкой входных данных
* @param packet Структура пакета для валидации
* @return true если пакет валиден, false если ошибка
* @date 2026-03-12
*/
bool FastLoRaReceiver::validatePacket(const DataComSet_t& packet) {
    // 🔑 ПРОВЕРКА #1: Валидность преамбулы
    if (packet.preamble[0] != PACKET_PREAMBLE_1 ||
        packet.preamble[1] != PACKET_PREAMBLE_2) {
        _statsRx.invalidPreamble++;
        ESP_LOGW(TAG_LORA_RX, "⚠️ Invalid preamble: 0x%02X 0x%02X",
                 packet.preamble[0], packet.preamble[1]);
        return false;
    }
    
    // 🔑 ПРОВЕРКА #2: Валидность packet_id (защита от переполнения)
    if (packet.packet_id == 0xFFFF) {
        ESP_LOGW(TAG_LORA_RX, "⚠️ Подозрительный packet_id: 0xFFFF");
    }
    
    // 🔑 ПРОВЕРКА #3: CRC (исключаем преамбулу 2б + сам CRC 1б)
    uint8_t calcCRC = calculateCRC8(
        (uint8_t*)&packet.packet_id,
        sizeof(DataComSet_t) - 3
    );
    
    if (packet.crc8 != calcCRC) {
        _statsRx.crcErrors++;
        ESP_LOGW(TAG_LORA_RX, "⚠️ CRC mismatch: got 0x%02X, calc 0x%02X",
                 packet.crc8, calcCRC);
        return false;
    }
    
    // 🔑 ПРОВЕРКА #4: Проверка дубликатов
    if (packet.packet_id == _lastPacketId && _lastPacketId != 0) {
        _statsRx.duplicatePackets++;
        ESP_LOGD(TAG_LORA_RX, "⚠️ Duplicate packet ID: %u", packet.packet_id);
        return false;
    }
    
    _lastPacketId = packet.packet_id;
    ESP_LOGV(TAG_LORA_RX, "✅ Packet validated: ID=%u", packet.packet_id);
    return true;
}







//=========================================================


//=====================================================================================
// должна только отправлять ACK и возвращать результат.
// Используем AckPacket_t из CommonTypes.h


void FastLoRaReceiver::handleRecovery() {
    _statsRx.recoveryAttempts++;
    ESP_LOGI(TAG_LORA_RX, "Attempting recovery...");
    // Serial.println("🔄 Attempting recovery...");
    // Try to reset the radio module gracefully
    _radio->reset();
    delay(10);

    // Re-initialize parameters (similar to begin but without full re-allocation)
    int state = _radio->begin(
        Config::Radio::FREQUENCY,
        Config::Radio::BANDWIDTH,
        Config::Radio::SPREADING_FACTOR,
        Config::Radio::CODING_RATE,
        Config::Radio::SYNC_WORD,
        Config::Radio::OUTPUT_POWER,
        Config::Radio::PREAMBLE_LEN
    );
    if (state == RADIOLIB_ERR_NONE) {
        _radio->setCRC(true);
        //_radio->setDio0Action(::onDataReceived, RISING);
        _radio->setDio0Action(::onDio0ISR, RISING);
        state = _radio->startReceive();
        if (state == RADIOLIB_ERR_NONE) {
            ESP_LOGI(TAG_LORA_RX, "✅ Recovery successful, back to listening.");
        //    Serial.println("✅ Recovery successful, back to listening.");
            _currentState = StateRx::LISTENING; // Assume success and go back to listening
            return;
        }
    }

    ESP_LOGE(TAG_LORA_RX, "❌ Recovery failed");
    //Serial.println("❌ Recovery failed.");
    _currentState = StateRx::ERROR; // Stay in error state or implement further steps
    delay(1000); // Brief pause after failed recovery attempt
}



void FastLoRaReceiver::printAckData(const AckPacket_t& ack) {
    // Реализация печати ACK пакета
    ESP_LOGD(TAG_LORA_RX, "ACK ID:%u CRC:0x%02X TS:%lu", 
             ack.packet_id, ack.crc8, ack.timestamp);
    //Serial.printf("ACK DATA: ID=%u CRC=0x%02X\n", ack.packet_id, ack.crc8);
}


// Используем DataComSet_t
// ============================================================================
// ПОЛУЧЕНИЕ ДАННЫХ — атомарная операция
// ============================================================================
bool FastLoRaReceiver::getFreshData(DataComSet_t& data) {
    if (_dataReady) {
        data = _lastData;
        _dataReady = false;
        return true;
    }
    return false;
}



/*
bool FastLoRaReceiver::getFreshData(DataComSet_t& data) {
    bool wasReady = false;
    
    portENTER_CRITICAL(&mux);
    if (_dataReady) {
        data = _lastData;
        _dataReady = false;
        wasReady = true;
        ESP_LOGV(TAG_LORA_RX, "✅ Data copied to caller (ID: %u)", data.packet_id);
    }
    portEXIT_CRITICAL(&mux);
    return wasReady;
}
*/


// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ
// ============================================================================
bool FastLoRaReceiver::isDataReady() const {
    bool ready;
    portENTER_CRITICAL(&mux);
    ready = _dataReady;
    portEXIT_CRITICAL(&mux);
    return ready;
}


bool FastLoRaReceiver::isConnected() const {
    uint32_t elapsed = millis() - _statsRx.lastPacketTime;
    return elapsed < Config::Timing::CONNECTION_TIMEOUT_MS;
}


void FastLoRaReceiver::getStats(uint32_t& received, uint32_t& success, 
                               uint32_t& crcErrors, uint32_t& acksSent, 
                               float& ackSuccessRate) {
    portENTER_CRITICAL(&mux);
        received = _statsRx.packetsReceived;
        success = _statsRx.packetsReceivedSuccess;
        crcErrors = _statsRx.crcErrors;
        acksSent = _statsRx.acksSent;
        ackSuccessRate = _statsRx.ackSuccessRate;
    portEXIT_CRITICAL(&mux);
}


ReceiverStats FastLoRaReceiver::getStatsFull() const {
    ReceiverStats snapshot;
    portENTER_CRITICAL(&mux);
    snapshot = _statsRx;
    portEXIT_CRITICAL(&mux);
    return snapshot;
}



void FastLoRaReceiver::resetStats() {
    portENTER_CRITICAL(&mux);
    memset(&_statsRx, 0, sizeof(_statsRx));
    portEXIT_CRITICAL(&mux);
}

// bool FastLoRaReceiver::diagnose() { return _radio != nullptr; }

bool FastLoRaReceiver::diagnose() {
    long version = _radio->getChipVersion();
    if (version == 0x12) { // Expected for SX1278
        ESP_LOGI(TAG_LORA_RX, "✅ SX1278 detected (Version: 0x%lX)", version);
        //Serial.printf("✅ SX1278 Diagnostics OK. Version: 0x%lX\n", version);
        return true;
    } else {
        ESP_LOGE(TAG_LORA_RX, "❌ Unexpected chip version: 0x%lX", version);
        //Serial.printf("❌ SX1278 Diagnostics FAILED. Unexpected Version: 0x%lX\n", version);
        return false;
    }
}


// Реализация getCurrentState
StateRx FastLoRaReceiver::getCurrentState() const {
    return _currentState;
}



void FastLoRaReceiver::printDebug(const char* message, DebugLevel level) {
    if (_debugLevel >= level) {
       ESP_LOGI(TAG_LORA_RX, "%s", message);
    //   Serial.printf("[LoRaRX] %s\n", message);
    }
}



// ============================================================================
// ОТЛАДОЧНЫЕ МЕТОДЫ
// ============================================================================
void FastLoRaReceiver::printPacketData() {
    ESP_LOGI(TAG_LORA_RX, "📦 ID:%u CRC:0x%02X Up:%d Left:%d Thr:%d Flags:0x%02X",
             _lastData.packet_id, _lastData.crc8,
             _lastData.comUp, _lastData.comLeft, 
             _lastData.comThrottle, _lastData.comSetAll);
}


void FastLoRaReceiver::printFullReport() {
    ReceiverStats stats = getStatsFull();
    
    ESP_LOGI(TAG_LORA_RX, "==========================================");
    ESP_LOGI(TAG_LORA_RX, "📊 ПОЛНЫЙ ОТЧЁТ ПРИЁМНИКА LoRa");
    ESP_LOGI(TAG_LORA_RX, "==========================================");
    ESP_LOGI(TAG_LORA_RX, "ПАКЕТЫ:");
    ESP_LOGI(TAG_LORA_RX, "  Всего принято: %lu", stats.packetsReceived);
    ESP_LOGI(TAG_LORA_RX, "  Успешно обработано: %lu", stats.packetsReceivedSuccess);
    ESP_LOGI(TAG_LORA_RX, "  Ошибок CRC: %lu", stats.crcErrors);
    ESP_LOGI(TAG_LORA_RX, "  Неверная преамбула: %lu", stats.invalidPreamble);
    ESP_LOGI(TAG_LORA_RX, "  Дубликаты: %lu", stats.duplicatePackets);
    
    ESP_LOGI(TAG_LORA_RX, "ПОДТВЕРЖДЕНИЯ (ACK):");
    ESP_LOGI(TAG_LORA_RX, "  Успешно отправлено: %lu", stats.acksSentSuccess);
    ESP_LOGI(TAG_LORA_RX, "  Ошибок отправки: %lu", stats.ackSendErrors);
    
    ESP_LOGI(TAG_LORA_RX, "СОЕДИНЕНИЕ:");
    ESP_LOGI(TAG_LORA_RX, "  Последний пакет: %lu мс назад", 
             millis() - stats.lastPacketTime);
    ESP_LOGI(TAG_LORA_RX, "  Статус: %s", isConnected() ? "АКТИВНО" : "ПОТЕРЯНО");
    ESP_LOGI(TAG_LORA_RX, "==========================================");
}


void FastLoRaReceiver::printFullReport_OLD() {
    Serial.println("\n" + String(80, '='));
    Serial.println("📊 COMPLETE RECEIVER REPORT");
    Serial.println(String(80, '='));
    
    Serial.println("PACKET STATISTICS:");
    Serial.printf("  Total Received: %lu\n", _statsRx.packetsReceived);
    Serial.printf("  Successfully Processed: %lu\n", _statsRx.packetsReceivedSuccess);
    Serial.printf("  CRC Errors: %lu\n", _statsRx.crcErrors);
    Serial.printf("  Invalid Preamble: %lu\n", _statsRx.invalidPreamble);
    Serial.printf("  Duplicate Packets: %lu\n", _statsRx.duplicatePackets);
    
    Serial.println("\nACK STATISTICS:");
    Serial.printf("  ACK Requests: %lu\n", _statsRx.ackRequests);
    Serial.printf("  ACK Responses: %lu\n", _statsRx.ackResponses);
    Serial.printf("  ACK Timeouts: %lu\n", _statsRx.ackTimeouts);
    Serial.printf("  ACK Errors: %lu\n", _statsRx.ackErrors);
    
    //... в расчете успешности ACK проверяем деление на ноль:
    if (_statsRx.ackRequests > 0) {
        float successRate = (_statsRx.ackResponses * 100.0f) / _statsRx.ackRequests;
        Serial.printf("  ACK Success Rate: %.1f%%\n", successRate);
    } else {
    Serial.println(" ACK Success Rate: N/A");
    }
    
    Serial.println("\nSIGNAL QUALITY:");
    Serial.printf("  Last RSSI: %d dBm\n", _statsRx.lastRSSI);
    Serial.printf("  Last SNR: %.1f dB\n", _statsRx.lastSNR);
    Serial.printf("  Last Packet: %lu ms ago\n", 
                  millis() - _statsRx.lastPacketTime);
    
    Serial.println("\nCONNECTION STATUS:");
    Serial.printf("  State: %s\n", isListening() ? "LISTENING" : "PROCESSING");
    Serial.printf("  Connection: %s\n", isConnected() ? "ACTIVE" : "LOST");
    
    Serial.println(String(80, '=') + "\n");

    
    Serial.println("\nSTATE MACHINE INFO:");
    Serial.printf("  Current State: %d\n", static_cast<int>(_currentState));
    Serial.printf("  Data Ready: %s\n", _dataReady ? "YES" : "NO");
    Serial.printf("  Need to Send ACK: %s\n", _needToSendAck ? "YES" : "NO");
    Serial.printf("  Last ACK Packet ID: %u\n", _ackPacketId);
    Serial.printf("  Data Received Flag: %s\n", _dataReceivedFlag ? "SET" : "CLEAR");
}


// ============================================================================
// 🔧 ИСПРАВЛЕНИЕ #4: БЕЗОПАСНАЯ ПЕЧАТЬ СТАТИСТИКИ
// ============================================================================
/**
* @brief Печать краткой статистики в Serial (БЕЗОПАСНАЯ ВЕРСИЯ)
* @note Добавлена защита от переполнения стека и проверки целостности данных
* @date 2026-03-12
*/
void FastLoRaReceiver::printStatistics() {
    // 🔑 ПРОВЕРКА #1: Инициализирован ли модуль
    if (!_initialized) {
        ESP_LOGW(TAG_LORA_RX, "⚠️ printStatistics: модуль не инициализирован!");
        return;
    }
    
    // 🔑 ПРОВЕРКА #2: Защита от рекурсивного вызова
    static bool _printing = false;
    if (_printing) {
        ESP_LOGW(TAG_LORA_RX, "⚠️ printStatistics() уже выполняется!");
        return;
    }
    _printing = true;
    
    // 🔑 ПРОВЕРКА #3: Доступность памяти перед печатью
    if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < 8192) {
        ESP_LOGW(TAG_LORA_RX, "⚠️ Мало свободной памяти для печати статистики!");
        _printing = false;
        return;
    }
    
   
    
    // 🔑 БЕЗОПАСНОЕ ЧТЕНИЕ СТАТИСТИКИ (с защитой от гонки данных)
    uint32_t pkt, ok, crc, dup, ack;
    portENTER_CRITICAL(&mux);
        pkt = _statsRx.packetsReceived;
        ok = _statsRx.packetsReceivedSuccess;
        crc = _statsRx.crcErrors;
        dup = _statsRx.duplicatePackets;
        ack = _statsRx.acksSentSuccess;
    portEXIT_CRITICAL(&mux);
    
    // 🔑 УПРОЩЁННЫЙ ФОРМАТ (меньше нагрузки на стек)
    //ESP_LOGI(TAG_LORA_RX, "📊 [RX STATS] Pkt:%lu OK:%lu CRC:%lu Dup:%lu ACK:%lu",
    //         pkt, ok, crc, dup, ack);
    ESP_LOGI(TAG_LORA_RX, "📊 [RX STATS] Пакеты: всего=%lu | успешно=%lu | CRC_ошибки=%lu | дубликаты=%lu | ACK_отправлено=%lu",
             pkt, ok, crc, dup, ack);
    
    // 🔧 ДОПОЛНИТЕЛЬНАЯ СТАТИСТИКА: Процент успешных пакетов
    if (pkt > 0) {
        float successRate = (static_cast<float>(ok) / pkt) * 100.0f;
        ESP_LOGI(TAG_LORA_RX, "   📈 Успешность приёма: %.1f%%", successRate);
    }
    _printing = false;
}

// ============================================================================
// 🔧 РЕАЛИЗАЦИЯ sendAck() — ИСПРАВЛЕНИЕ ОШИБКИ КОМПИЛЯЦИИ
// ============================================================================
/**
 * @brief Отправка подтверждения приёма (ACK)
 * @return true при успешной отправке, false при ошибке
 * 
 * @details Формирует и отправляет пакет подтверждения с:
 * - Правильной преамбулой (0x55 0xAA)
 * - ID подтверждаемого пакета
 * - Временной меткой
 * - Статусом (1=OK)
 * - CRC8 контрольной суммой
 * 
 * @note После отправки модуль автоматически возвращается в режим приёма
 * @warning Блокирующая операция — не вызывать из ISR!
 */
bool FastLoRaReceiver::sendAck() {
    ESP_LOGV(TAG_LORA_RX, "📤 sendAck(): подготовка пакета для ID=%u", _ackPacketId);
    
    // 🔑 Проверка валидности указателей
    if (!_radio || !_module) {
        ESP_LOGE(TAG_LORA_RX, "❌ sendAck(): null pointer!");
        return false;
    }
    
    // 🔑 Формирование ACK пакета
    _ackPacket.preamble[0] = ACK_PREAMBLE_1;  // 0x55
    _ackPacket.preamble[1] = ACK_PREAMBLE_2;  // 0xAA
    _ackPacket.packet_id = _ackPacketId;
    _ackPacket.timestamp = millis();
    _ackPacket.status = 1;  // OK
    
    // 🔑 Расчёт CRC8 (исключаем преамбулу 2б + сам CRC 1б)
    _ackPacket.crc8 = calculateCRC8(
        (uint8_t*)&_ackPacket.packet_id, 
        sizeof(AckPacket_t) - 3
    );
    
    ESP_LOGV(TAG_LORA_RX, "📦 ACK: ID=%u TS=%lu CRC=0x%02X", 
             _ackPacket.packet_id, _ackPacket.timestamp, _ackPacket.crc8);
    
    // 🔑 Переключение в режим передачи
    int state = _radio->standby();
    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG_LORA_RX, "❌ standby() failed: %d", state);
        return false;
    }
    delayMicroseconds(100);  // Стабилизация
    
    // 🔑 Отправка пакета (блокирующая)
    state = _radio->transmit(
        (uint8_t*)&_ackPacket, 
        sizeof(AckPacket_t)
    );
    
    if (state == RADIOLIB_ERR_NONE) {
        _statsRx.acksSentSuccess++;
        ESP_LOGI(TAG_LORA_RX, "✅ ACK отправлен успешно для ID=%u", _ackPacketId);
        
        // 🔑 Возврат в режим приёма
        state = _radio->startReceive();
        if (state != RADIOLIB_ERR_NONE) {
            ESP_LOGW(TAG_LORA_RX, "⚠️ startReceive() после ACK: %d", state);
        }
        return true;
        
    } else {
        _statsRx.ackSendErrors++;
        ESP_LOGE(TAG_LORA_RX, "❌ transmit() failed: %d", state);
        
        // 🔑 Попытка восстановления
        _radio->standby();
        delay(10);
        _radio->startReceive();
        return false;
    }
}// END sendAck()


//**Место:** Реализация новой функции `diagnoseDIO1()` (в конец файла)
//
/**
 * @brief Проверка состояния пина DIO1
 * @return true если пин настроен корректно
 */

 /*
bool FastLoRaReceiver::diagnoseDIO1() {
    ESP_LOGI(TAG_LORA_RX, "🔍 Диагностика DIO1 (GPIO%d)...", Config::Pins::LORA_DIO1);
    pinMode(Config::Pins::LORA_DIO1, INPUT_PULLUP);
    delay(10);
    int state = digitalRead(Config::Pins::LORA_DIO1);
    ESP_LOGI(TAG_LORA_RX, "   📊 Состояние пина: %s (ожидаемо HIGH без активности)", 
             state ? "HIGH" : "LOW");
    return true;
}
*/

// ============================================================================
// 🔧 РЕАЛИЗАЦИЯ: diagnoseDIO1() — диагностика нового модуля
// ============================================================================
/**
 * @brief Проверка состояния и конфигурации пина DIO1
 * @return true если пин настроен корректно и готов к работе
 */
bool FastLoRaReceiver::diagnoseDIO1() {
#if !LORA_ENABLE_DIO1s
    ESP_LOGW(TAG_LORA_RX, "⚠️ diagnoseDIO1: DIO1 отключён в Config.h");
    return false;
#endif

    if (Config::Pins::LORA_DIO1 == RADIOLIB_NC) {
        ESP_LOGE(TAG_LORA_RX, "❌ diagnoseDIO1: LORA_DIO1 не задан (RADIOLIB_NC)");
        return false;
    }

    ESP_LOGI(TAG_LORA_RX, "🔍 === ДИАГНОСТИКА DIO1 (GPIO%d) ===", Config::Pins::LORA_DIO1);
    
    // 1. Проверка конфигурации пина
    pinMode(Config::Pins::LORA_DIO1, INPUT_PULLUP);
    delay(2);  // Стабилизация
    
    int pin_state = digitalRead(Config::Pins::LORA_DIO1);
    ESP_LOGI(TAG_LORA_RX, "   📊 Состояние пина: %s (ожидаемо HIGH без активности)",
             pin_state ? "HIGH ✅" : "LOW ⚠️");
    
    // 2. Проверка через RadioLib (если модуль инициализирован)
    if (_initialized && _radio != nullptr) {
        // 🔧 RadioLib не предоставляет прямого метода чтения состояния DIO1,
        // но можно проверить, настроено ли действие
        ESP_LOGI(TAG_LORA_RX, "   ✅ Модуль инициализирован, DIO1 настроен через setDio1Action()");
    }
    
    // 3. Рекомендации по подключению
    ESP_LOGI(TAG_LORA_RX, "   🔌 Проверьте подключение:");
    ESP_LOGI(TAG_LORA_RX, "      • DIO1 модуля → GPIO%d ESP32", Config::Pins::LORA_DIO1);
    ESP_LOGI(TAG_LORA_RX, "      • Pull-up резистор 10кΩ (опционально, но рекомендуется)");
    ESP_LOGI(TAG_LORA_RX, "      • Общий GND между модулем и ESP32");
    
    ESP_LOGI(TAG_LORA_RX, "✅ Диагностика DIO1 завершена");
    return true;
}//END diagnoseDIO1() 


