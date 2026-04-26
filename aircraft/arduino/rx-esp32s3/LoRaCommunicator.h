


/**
* @file LoRaCommunicator.h
* @brief Высокоскоростная неблокирующая LoRa связь с подтверждением приема (исправленная версия)
* @version 5.1.1 — ИСПРАВЛЕНО: архитектура класса, обработка прерываний, дублирование объявлений
* @date 2026-02-16
*/
/**
 * @file LoRaCommunicator.h
 * @brief Заголовочный файл для высокоскоростного LoRa коммуникатора
 * 
 * @details Реализует неблокирующий приёмник SX1278 с конечным автоматом (FSM) и 
 * подтверждением приёма пакетов (ACK). Основные особенности:
 * - Поддержка параметров из Config.h
 * - Неблокирующая архитектура с update()
 * - Обработка ошибок с подробной диагностикой
 * - Система подтверждения приёма (ACK)
 * 
 * @note Критически важно: Module создаётся ВНЕ класса (в main/Receiver.ino)
 * и передаётся по ссылке в конструктор.
 * 
 * @author Embedded Systems Team
 * @version 5.5
 * @date 2026-02-18
 */
/**
 * @file LoRaCommunicator.h
 * @brief Высокоскоростная неблокирующая LoRa связь с подтверждением приема (исправленная версия)
 * @version 5.2.0 — ИСПРАВЛЕНО: StateRx enum, добавлена поддержка DIO1 для Receiver, исправлены типы
 * @date 2026-04-07
 */

#pragma once
#ifndef LORA_COMMUNICATOR_H
#define LORA_COMMUNICATOR_H

#include <SPI.h>
#include <Arduino.h>
#include "esp_log.h"
#include <cstdint>
#include <RadioLib.h>
#include "CommonTypes.h"
#include "Config.h"

// ============================================================================
// ENUMS И STATES
// ============================================================================

/**
 * @brief Состояния конечного автомата приёмника
 */
enum class StateRx : uint8_t {
    INIT          = 0,    ///< Инициализация
    LISTENING     = 1,    ///< Ожидание данных (Приём)
    PROCESSING    = 2,    ///< Обработка данных
    SENDING_ACK   = 3,    ///< Отправка подтверждения
    ERROR         = 4     ///< Состояние ошибки
};

// ============================================================================
// STRUCTS
// ============================================================================

/**
 * @brief Статистика приёмника
 */
struct ReceiverStats {
    uint32_t packetsReceived = 0;
    uint32_t packetsReceivedSuccess = 0;
    uint32_t crcErrors = 0;
    uint32_t invalidPreamble = 0;
    uint32_t duplicatePackets = 0;
    uint32_t packetErrors = 0;
    uint32_t lastPacketTime = 0;
    int16_t lastRSSI = 0;
    float lastSNR = 0.0f;
    uint32_t fsmErrors = 0;
    uint32_t recoveryAttempts = 0;
    
    // ACK stats
    uint32_t acksSent = 0;
    uint32_t acksSentSuccess = 0;
    uint32_t ackSendErrors = 0;
    uint32_t ackRequests = 0;
    uint32_t ackResponses = 0;
    uint32_t ackErrors = 0;
    uint32_t ackTimeouts = 0;
    float ackSuccessRate = 0.0f;
    
    // 🆕 ДОБАВЛЕНО: Счетчик таймаутов приема (DIO1)
    uint32_t receive_timeouts = 0; 
};





// ============================================================================
// КЛАСС ПРИЕМНИКА LoRa (SX1278) — ИСПРАВЛЕННАЯ АРХИТЕКТУРА
// ============================================================================
/**
 * @class FastLoRaReceiver
 * @brief Неблокирующий приёмник SX1278 с FSM и ACK
 * 
 * @details Класс реализует неблокирующую обработку LoRa-пакетов с использованием
 * конечного автомата. Важные особенности:
 * - Module создаётся ВНЕ класса и передаётся по ссылке
 * - Поддержка перезапуска через restart()
 * - Подробная диагностика ошибок через ESP_LOG
 * - Система подтверждения приёма (ACK)
 * 
 * @note Критически важно: Все пины зафиксированы в Config.h и не должны меняться
 *
 * 🔑 КРИТИЧЕСКИ: Module создаётся ВНЕ класса (в main/Receiver.ino)
 *              и передаётся по ссылке в конструктор.
 */
/**
 * @class FastLoRaReceiver
 * @brief Неблокирующий приёмник SX1278 с FSM и ACK
 */
class FastLoRaReceiver {
public:
    /**
     * @brief Конструктор приемника с условной поддержкой DIO1
     * @param cs_pin Пин Chip Select
     * @param rst_pin Пин Reset
     * @param dio0_pin Пин DIO0 (обязателен)
     * @param dio1_pin Пин DIO1 (требуется при LORA_ENABLE_DIO1=1)
     * @param spi Указатель на SPI-шину
     */
    FastLoRaReceiver(
        uint8_t cs_pin,
        uint8_t rst_pin,
        uint8_t dio0_pin,
        int16_t dio1_pin,     // 🔑 Только если включена поддержка DIO1
        SPIClass* spi         // ← Должен быть указатель!
    );

    ~FastLoRaReceiver();
    
    /**
     * @brief Инициализация модуля SX1278 с параметрами из конфигурации
     *
     * @details Выполняет полную инициализацию радиомодуля с детальной диагностикой.
     *
     * Может быть вызван повторно для перезапуска модуля.
     * 
     * @return true при успешной инициализации, false при ошибке
     */
    bool begin();

    /**
    * @brief Основной метод обновления состояния конечного автомата
    * 
    * @details Вызывать регулярно в loop(). Обрабатывает приём данных,
    * состояние модуля и отправку ACK при необходимости.
     */
    void update();
    
    /**
    * @brief Получение свежих данных при их наличии
    * @note Атомарная операция с использованием мьютекса
    * @param data Структура для сохранения полученных данных
    * @return true если данные были скопированы, false если данных нет
    * @warning После вызова флаг _dataReady сбрасывается в false
    * @see getFreshData - основной метод получения данных
    */
    bool getFreshData(DataComSet_t& data); 

    
    /**
     * @brief Проверка успешной инициализации радиомодуля
     * @return true если радиомодуль инициализирован успешно
     */
    bool isInitialized() const { return _initialized; }
    
    /**
     * @brief Проверка готовности новых данных
     * @return true если данные готовы для получения
     */
    bool isDataReady() const;
    
    /**
     * @brief Получение текущего состояния FSM
     * @return Текущее состояние автомата
     */
    StateRx getCurrentState() const;
    
    /**
     * @brief Проверка активности соединения
     * @return true если соединение активно (последний пакет < 5 сек)
     */
    bool isConnected() const;
    
    /**
     * @brief Сброс статистики
     */
    void resetStats();
    
    /**
     * @brief Печать краткой статистики в Serial
     */
    void printStatistics();
    
    /**
     * @brief Печать полного отчёта в Serial
     */
    void printFullReport();
    
    /**
     * @brief Установка уровня отладки
     * @param level Желаемый уровень детализации
     */
    void setDebugLevel(DebugLevel level);
    
    /**
     * @brief Получение текущего уровня отладки
     * @return Текущий уровень
     */
    DebugLevel getDebugLevel() const;
    
    /**
     * @brief Получение полной статистики (потокобезопасно)
     * @return Копия структуры статистики
     */
    ReceiverStats getStatsFull() const;

    void getStats(uint32_t& received, uint32_t& success, 
                               uint32_t& crcErrors, uint32_t& acksSent, 
                               float& ackSuccessRate);

    /**
     * @brief Диагностика радиомодуля  (проверка версии чипа)
     * @return true если модуль отвечает корректно
     */
    bool diagnose();
   
    /**
    * @brief Перезапуск радиомодуля в случае ошибки
    * 
    * @details Попытка восстановить работу радиомодуля после сбоя.
    * Выполняет полную повторную инициализацию.
    * 
     * 
     * @return true при успешном перезапуске, false при ошибке
     */
    bool restart();
    
    
    
    /**
     * @brief   // Статический указатель для доступа из прерывания 
     * (публичный для безопасности)
     * 
     */
    //  static FastLoRa Receiver* _instance;

    bool isListening() const { return _currentState == StateRx::LISTENING; }

    void getAckStats(uint32_t& requests, uint32_t& responses,
                     uint32_t& timeouts, uint32_t& errors);

    /**
     * @brief Диагностика SPI-соединения с SX1278
     * @return true если модуль отвечает корректно
     */
    bool diagnoseSPI();  // ← Теперь публичный!
    
    // ============================================================================
    // 🔧 ДОБАВИТЬ: Диагностика DIO1 (для нового модуля с DIO1)
    // ============================================================================
    /**
    * @brief 🔑 Диагностика активности пина DIO1 (RadioLib 7.6.0+)
    * @note SX1278 использует DIO1 для:
    *   • Rx Timeout (приём не начался за заданное время)
    *   • Завершения CAD (Channel Activity Detection)
    * @return true если пин настроен корректно
    */
    bool diagnoseDIO1();  // ← 🔑 ДОБАВИТЬ ОБЪЯВЛЕНИЕ

    // ============================================================================
    // 🔧 ДОБАВИТЬ ОБЪЯВЛЕНИЯ МЕТОДОВ ОБРАБОТКИ ПРЕРЫВАНИЙ
    // ============================================================================
    /**
    * @brief Обработчик прерывания RxDone (DIO0)
    * @note Объявлен с IRAM_ATTR для размещения в быстрой памяти
    */
    void onRxDone() IRAM_ATTR;

    /**
    * @brief Обработчик прерывания RxTimeout (DIO1)
    * @note Требуется для модулей с поддержкой DIO1 (RadioLib 7.6.0+)
    */
    void onRxTimeout() IRAM_ATTR;  // ← 🔑 ДОБАВИТЬ ЭТУ СТРОКУ!

private:
    // Указатели на внешние объекты (НЕ владеем памятью)
    // --- RADIO OBJECTS ---
    Module* _module;          ///< Указатель на внешний объект Module
    SX1278* _radio;           ///< Указатель на объект радиомодуля
    SPIClass* _spi;         //  = nullptr; ///< Указатель на объект SPI Шины

    // --- INSTANCE & STATE ---
    static FastLoRaReceiver* _instance;
    StateRx _currentState  = StateRx::INIT;    ///< Текущее состояние конечного автомата

    bool _initialized;                      //  = false; ///< Флаг успешной инициализации

    // > 📝 **Документация:** Поле `_chipConnected` используется для отслеживания 
    // успешного обнаружения SX1278 по SPI. Инициализируется в конструкторе и 
    // проверяется в `diagnoseSPI()`.
    bool _chipConnected;                // = false;  ///< Флаг подключения чипа по SPI

    // --- FLAGS (Volatile for ISR) ---
    volatile bool _dataReady = false;
    volatile bool _dataReceivedFlag = false;  ///< Флаг прерывания DIO0
    volatile bool _dio1TimeoutFlag = false;   ///< 🆕 Флаг: сработал Rx Timeout (DIO1)
    volatile bool _dio1CadFlag = false;       ///< 🆕 Флаг: обнаружение канала (CAD)

   // Состояние и флаги (атомарные для прерываний)
    // Флаги (volatile для ISR)
    // ============================================================================
    // 🔧 ДОБАВИТЬ: Флаги и обработчик для DIO1 (RadioLib 7.6.0+)
    // Файл: LoRaCommunicator.h | Класс: FastLoRaReceiver | Секция: private
    // ============================================================================

    // --- DATA BUFFERS ---
    DataComSet_t _receivedData ; ///< Буфер полученных данных
    uint32_t _lastReceiveTime ;  ///< Время приема последнего пакета
    uint32_t _errorCount ;       ///< Счетчик ошибок
    uint32_t _lastErrorTime ;    ///< Время последней ошибки

    ReceiverStats _packetStats; ///< Статистика работы радиомодуля

    // volatile bool _dataReady = false;         // Флаг готовности данных для внешнего кода
    volatile uint16_t _ackPacketId;             // = 0;  // ID пакета для отправки ACK
    
    // Данные
    DataComSet_t _lastData{};                 ///< Последний принятый пакет
    AckPacket_t _ackPacket{};                 ///< Буфер для формирования ACK
    uint16_t _lastPacketId;     //  = 0;      ///< ID последнего обработанного пакета
    
    // Конфигурация
    LoRaConfig_t _config;
    
    // Отладка
    DebugLevel _debugLevel;     //  = DebugLevel::DEBUG_NONE;
    
    // Статистика (защищена мьютексом)
    mutable ReceiverStats _statsRx;
    
    // FSM методы
    void fsmInit();
    void fsmListening();
    void fsmProcessing();
    void fsmSendingAck();
    void fsmError();
    void handleRecovery();  // ← ДОБАВИТЬ ЭТУ СТРОКУ  
    
    /**
     * @brief Отправка подтверждения приема (ACK)
     * @param packetId Идентификатор пакета для подтверждения
     * @return true при успешной отправке
     */
    bool sendAckPacket(uint16_t packetId);
    

    // Вспомогательные методы
    bool validatePacket(const DataComSet_t& packet);
    //  void sendAck(uint16_t packetId);
    bool sendAck();
    void updateAckStats(bool success);
    volatile bool _needToSendAck = false;           ///< Флаг необходимости отправки ACK
    
    void updateState(StateRx newState);
    void printPacketData();
    void printAckData(const AckPacket_t& ack);
    void printDebug(const char* message, DebugLevel level = DebugLevel::DEBUG_INFO);
    
    void printFullReport_OLD();
    //  void printFullReport();
    // bool diagnoseSPI();  // ← Добавить объявление
    // Дружественная функция для доступа к приватным членам из прерывания
    // Friend для доступа из ISR
    
    // --- FRIENDS FOR ISR ACCESS ---
    //                    friend void IRAM_ATTR onDio0ISR();
    //                    friend void IRAM_ATTR onDio1ISR();

    // ========================================================================
    // 🔧 FRIEND ДЛЯ ISR (RadioLib 7.6.0+ с DIO0/DIO1)
    // ========================================================================
    /**
     * @brief Глобальные обработчики прерываний (доступ к приватным членам)
     * @note Реализации находятся в LoRaCommunicator.cpp как глобальные функции
     */
    friend void IRAM_ATTR onDio0ISR();   ///< Обработчик DIO0 (RxDone)
    friend void IRAM_ATTR onDio1ISR();   ///< Обработчик DIO1 (RxTimeout/CAD)


    // 🆕 ДОБАВИТЬ: Флаги прерываний
    volatile bool _rxDoneFlag = false;     // Флаг от DIO0
    volatile bool _rxTimeoutFlag = false;  // Флаг от DIO1 (если используется)

};

// Глобальные переменные для работы с прерываниями
// extern FastLoRaReceiver* receiver;


#endif // LORA_COMMUNICATOR_H