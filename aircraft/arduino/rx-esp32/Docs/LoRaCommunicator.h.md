
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

// Тег для логирования
// Лучше определить в .cpp, а здесь не определять вообще, чтобы избежать redefinition
// static const char* TAG_LORA_RX = "LORA_COMM";

  
// FSM States for Receiver
// Состояния конечного автомата приёмника
enum class StateRx : uint8_t {
    //            INIT = 0,
    //            LISTENING = 1,
    //            PROCESSING = 2,
    //            SENDING_ACK = 3,
    //            ERROR = 4
  INIT          = 0,    ///< Инициализация
//  WAITING       = 1,    ///< Ожидание данных
  RECEIVING     = 1,    ///< Приём данных
  PROCESSING    = 2,    ///< Обработка данных
  SENDING_ACK   = 3,    ///< Отправка подтверждения
  ERROR         = 4     ///< Состояние ошибки
};



// Статистика приёмника
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
    
    // 🔧 ДОБАВИТЬ отсутствующее поле:
    //uint32_t receive_timeouts = 0;  ///< Счётчик таймаутов приёма (DIO1)
    uint32_t receive_timeouts = 0; // 🆕 ДОБАВИТЬ: Счетчик таймаутов приема (DIO1)

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
class FastLoRaReceiver {
public:
    /**
    * @brief Конструктор приёмника с внешним объектом Module RadioLib
    * @param module_ref Ссылка на внешний объект Module (должен существовать всё время работы)
    *
    */
    // === КОНСТРУКТОР С ПЕРЕДАЧЕЙ SPI ===
    /**
     * @brief Конструктор приемника с условной поддержкой DIO1
     * @param cs_pin Пин Chip Select
     * @param rst_pin Пин Reset
     * @param dio0_pin Пин DIO0 (обязателен)
     * @if LORA_ENABLE_DIO1
     * @param dio1_pin Пин DIO1 (требуется при LORA_ENABLE_DIO1=1)
     * @endif
     * @param spi Указатель на SPI-шину
     * @param packetBuilder Указатель на билдер пакетов
     */
    FastLoRaReceiver(
        uint8_t cs_pin, 
        uint8_t rst_pin,
        uint8_t dio0_pin, 
      #if LORA_ENABLE_DIO1
          int16_t dio1_pin,          // 🔑 Только если включена поддержка DIO1
      #endif
        SPIClass* spi,
    //    DataPacketBuilder* packetBuilder  // ← новое
         );
    //explicit FastLoRaReceiver(Module& module_ref);
    // FastLoRaReceiver(Module& module_ref);

    // 🆕 ДОБАВИТЬ: Статические методы для ISR
    static void IRAM_ATTR onRxDone();   // DIO0: Пакет принят
    static void IRAM_ATTR onRxTimeout(); // DIO1: Таймаут приема (опционально, если используется)


   /**
   * @brief Деструктор
   */
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
    //  static FastLoRaReceiver* _instance;

    bool isListening() const { return _currentState == StateRx::LISTENING; }

    /**
     * @brief Диагностика SPI-соединения с SX1278
     * @return true если модуль отвечает корректно
     */
    bool diagnoseSPI();  // ← Теперь публичный!
    
    /**
     * @brief Диагностика активности пина DIO1 (RadioLib 7.6.0+)
     * @note SX1278 использует DIO1 для таймаута приема (Rx Timeout) 
     *       и корректного завершения сессии при потере сигнала.
     */
    bool diagnoseDIO1();

private:
    // Указатели на внешние объекты (НЕ владеем памятью)
    Module* _module;          ///< Указатель на внешний объект Module
    SX1278* _radio;           ///< Указатель на объект радиомодуля
    SPIClass* _spi = nullptr;
    StateRx _currentState;    ///< Текущее состояние конечного автомата
    static FastLoRaReceiver* _instance;
    bool _initialized;        ///< Флаг успешной инициализации
    bool _dataReady;          ///< Флаг наличия свежих данных

   // Состояние и флаги (атомарные для прерываний)
    // Флаги (volatile для ISR)
    // ============================================================================
    // 🔧 ДОБАВИТЬ: Флаги и обработчик для DIO1 (RadioLib 7.6.0+)
    // Файл: LoRaCommunicator.h | Класс: FastLoRaReceiver | Секция: private
    // ============================================================================
    // Флаги прерываний (volatile для ISR)
    volatile bool _dataReceivedFlag = false;  ///< Флаг прерывания DIO0 (пакет готов)
    volatile bool _dio1TimeoutFlag = false;   ///< 🆕 Флаг: сработал Rx Timeout (DIO1)
    volatile bool _dio1CadFlag = false;       ///< 🆕 Флаг: обнаружение канала (CAD)

    // 🔑 Дружественная функция для доступа из ISR DIO1
    friend void IRAM_ATTR onDio1ISR();

    DataComSet_t _receivedData; ///< Буфер полученных данных
    uint32_t _lastReceiveTime;  ///< Время последнего приёма
    uint32_t _errorCount;       ///< Счетчик ошибок
    uint32_t _lastErrorTime;    ///< Время последней ошибки

    ReceiverStats _packetStats; ///< Статистика работы радиомодуля

    // 🆕 Убедиться, что есть объявление ISR для DIO1 (Rx Timeout/CAD)
    // Если используется только DIO0 для RxDone, то DIO1 может использоваться для таймаута
    static void IRAM_ATTR onDio1ISR(); 


    // volatile bool _dataReady = false;         // Флаг готовности данных для внешнего кода
    volatile uint16_t _ackPacketId = 0;       // ID пакета для отправки ACK
    
    // Данные
    DataComSet_t _lastData{};                 ///< Последний принятый пакет
    AckPacket_t _ackPacket{};                 ///< Буфер для формирования ACK
    uint16_t _lastPacketId = 0;               ///< ID последнего обработанного пакета
    
   
    // Конфигурация
    LoRaConfig_t _config;
    
    // Отладка
    DebugLevel _debugLevel = DebugLevel::DEBUG_NONE;
    
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
    friend void IRAM_ATTR onDio0ISR();  // Доступ к приватным членам из прерывания
    //void IRAM_ATTR onDio0ISR();

    // 🆕 ДОБАВИТЬ: Флаги прерываний
    volatile bool _rxDoneFlag = false;     // Флаг от DIO0
    volatile bool _rxTimeoutFlag = false;  // Флаг от DIO1 (если используется)

};

// Глобальные переменные для работы с прерываниями
// extern FastLoRaReceiver* receiver;


#endif // LORA_COMMUNICATOR_H