// Проект Sender / Receiver (_SENDER_)
// Файл :: CommonTypes.h
//========================================
//  Шаг 2: Создаем файл CommonTypes.h
//========================================
// 2. Решение: Создаем полную реализацию
// 2.1. Обновленный CommonTypes.h
//

// CommonTypes.h

#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#pragma once
 
#include <Arduino.h>
#include <cstdint>
#include <esp_err.h>
#include <esp_log.h>
#define LOG_TAG "Receiver"

/**
 * @file CommonTypes.h
 * @brief Общие типы данных для Sender/Receiver проектов
 * 
 * Выбор структур данных обоснован:
 * - #pragma pack(push,1) гарантирует выравнивание для передачи по радио
 * - Статические assert проверяют совместимость размеров структур
 * - TimerMillis обеспечивает неблокирующие временные задержки
 */

// В CommonTypes.h добавить макросы
#define ESP_ERROR_CHECK_RETURN(x) do {        \
    esp_err_t __err_rc = (x);                 \
    if (__err_rc != ESP_OK) {                 \
        ESP_LOGE("ERROR", "Failed at %s:%d (%s)", __FILE__, __LINE__, esp_err_to_name(__err_rc)); \
        return false;                         \
    }                                         \
} while(0)
//
#define ESP_ERROR_CHECK_VOID(x) do {          \
    esp_err_t __err_rc = (x);                 \
    if (__err_rc != ESP_OK) {                 \
        ESP_LOGE("ERROR", "Failed at %s:%d (%s)", __FILE__, __LINE__, esp_err_to_name(__err_rc)); \
        return;                               \
    }                                         \
} while(0)

/**
 * @file CommonTypes.h
 * @brief Общие типы данных для Sender/Receiver проектов
 * 
 * ВАЖНО: Этот файл должен быть включен ВСЕМИ другими файлами
 * для обеспечения единой системы типов и констант.
 */
 


// ============================================================================
// КОНСТАНТЫ (ОПРЕДЕЛЯЮТСЯ ТОЛЬКО ЗДЕСЬ!)
// ============================================================================

// Флаги управления командами в comSetAll
#define ACK_REQUEST_FLAG   0b00000001  ///< Запрос подтверждения приема
#define PARASHUT_FLAG      0b00000010  ///< Активация парашюта
#define PHOTO_CARD_FLAG    0b00000100  ///< Запрос фотосъемки
#define GPS_DATA_FLAG      0b00001000  ///< Запрос GPS данных
#define SCREW_FLAG         0b00010000  ///< Команда поворота винтом
#define EMERGENCY_STOP     0b00100000  ///< Аварийная остановка
#define BATTERY_CHECK      0b01000000  ///< Запрос состояния батареи
#define SYSTEM_STATUS      0b10000000  ///< Запрос статуса системы

// Уровни детализации отладки
enum class DebugLevel : uint8_t  {
    DEBUG_NONE      = 0,  // Без вывода (минимальная нагрузка)
    DEBUG_ERRORS    = 1,  // Только критические ошибки
    DEBUG_WARN      = 2,  // Печать предупреждений по коду
    DEBUG_INFO      = 3,  // Печать информационных сообщений
    DEBUG_STATS     = 4,  // Ошибки + периодическая статистика
    DEBUG_PRINT     = 5,  // Печать отладочных сообщений Basic prints
    DEBUG_VERBOSE   = 6,  // Detailed prints
    DEBUG_ALL       = 7   // Полная отладка (все события)
};


#pragma pack(push, 1)

/**
 * @struct DataComSet_t
 * @brief Структура передаваемых данных управления (15 байт)
 */
struct DataComSet_t {
    uint8_t  preamble[2];    // 0xAA 0x55
    uint16_t packet_id;      // Счетчик пакетов (0-65535)
    uint8_t  comUp;          // Команда вверх 0-255
    uint8_t  comLeft;        // Команда влево 0-255  
    uint16_t comThrottle;    // Тяга двигателя 1000-2000
    uint8_t  comParashut;    // Парашют 0/1
    uint32_t timestamp;      // Временная метка (millis())
    uint8_t  comSetAll;      // Битовая маска команд
    uint8_t  crc8;           // Контрольная сумма
};

/**
 * @struct AckPacket_t  
 * @brief Структура подтверждения приема (10 байт)
 */
struct AckPacket_t {
    uint8_t preamble[2];     // 0x55 0xAA
    uint16_t packet_id;      // ID подтверждаемого пакета
    uint32_t timestamp;      // Время отправки ACK
    uint8_t status;          // Статус (1-OK, 0-ERROR)
    uint8_t crc8;            // Контрольная сумма
};

#pragma pack(pop)

// Размеры буферов
constexpr uint8_t MAX_PACKET_SIZE = sizeof(DataComSet_t);
constexpr uint8_t ACK_PACKET_SIZE = sizeof(AckPacket_t);

// Валидация структур для предотвращения ошибок
static_assert(sizeof(DataComSet_t) == 15, "Invalid DataComSet_t size");
static_assert(sizeof(AckPacket_t) == 10, "Invalid AckPacket_t size");


// CRC8
extern const uint8_t CRC8_TABLE[256];
uint8_t calculateCRC8(const uint8_t* data, size_t length);

// Отладка
void printDataComSet(const DataComSet_t& data, const char* source);


/**
 * @enum RadioStatus
 * @brief Статусы операций радио модуля
 * 
 * Замена RadioStatus для универсальности с RadioLib:
 * - SUCCESS/ERROR_* - стандартные статусы операций
 * - OPERATION_PENDING - для неблокирующих операций
 * - CONVERTED из ошибок RadioLib для единообразия
 */
enum class RadioStatus {
    SUCCESS,
    ERROR_INIT,
    ERROR_SEND,
    ERROR_RECEIVE,
    ERROR_CRC,
    ERROR_TIMEOUT,
    ERROR_NO_NEW_DATA,
    ERROR_INVALID_DATA,
    ERROR_INVALID_PREAMBLE,
    ERROR_BUSY,
    ERROR_NOT_READY,
    ERROR_CONNECTION_LOST,
    OPERATION_PENDING
};

/**
 * @class TimerMillis
 * @brief Неблокирующий таймер для замены delay()
 * 
 * Критически важен для системы реального времени:
 * - Избегает блокирующих вызовов delay()
 * - Безопасная работа с переполнением millis()
 * - Переиспользуется в multiple состояниях State Machine
 */
class TimerMillis {
private:
    uint32_t _lastTime = 0;
    uint32_t _interval = 1;
    
public:
    TimerMillis() = default;
    explicit TimerMillis(uint32_t interval) : _interval(interval) { reset(); }
    
    void reset() { _lastTime = millis(); }
    bool isReady() { 
        uint32_t current = millis();
        return (current - _lastTime) >= _interval; 
    }
    void setInterval(uint32_t interval) { _interval = interval; }
    uint32_t getElapsed() const { return millis() - _lastTime; }
    uint32_t getRemaining() const { 
        uint32_t elapsed = getElapsed();
        return (elapsed >= _interval) ? 0 : (_interval - elapsed);
    }
};

/**
 * @class BitMask
 * @brief Работа с битовыми масками для comSetAll
 * 
 * Позволяет устанавливать/читать отдельные биты:
 * - Бит 0: запрос ACK подтверждения
 * - Бит 1: запрос телеметрии
 * - Бит 2: запрос информации о батарее
 * - Остальные биты зарезервированы для будущего использования
 */
class BitMask {
private:
    uint8_t mask;
public:
    BitMask() : mask(0) {}
    explicit BitMask(uint8_t initialMask) : mask(initialMask) {}
    
    void setBit(uint8_t bit, bool value) {
        if (bit > 7) return;
        if (value) mask |= (1 << bit);
        else mask &= ~(1 << bit);
    }
    
    bool getBit(uint8_t bit) const {
        if (bit > 7) return false;
        return (mask & (1 << bit)) != 0;
    }
    
    uint8_t getMask() const { return mask; }
    void setMask(uint8_t newMask) { mask = newMask; }
};

// Внешнее объявление таблицы CRC
extern const uint8_t CRC8_TABLE[256];

/**
 * @brief Расчет CRC8 с использованием стандартной таблицы
 * @param data Указатель на данные
 * @param length Длина данных
 * @return Рассчитанный CRC8
 * 
 * Использует промышленный стандарт CRC-8 с полиномом 0x07:
 * - Высокая скорость благодаря табличному методу
 * - Хорошее обнаружение ошибок
 * - Совместимость с существующими системами
 */
uint8_t calculateCRC8(const uint8_t* data, size_t length);


#endif // COMMON_TYPES_H
//=========================================================================
