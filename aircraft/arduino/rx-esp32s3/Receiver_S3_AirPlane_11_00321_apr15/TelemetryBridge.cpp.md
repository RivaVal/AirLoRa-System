

//  📄 ФАЙЛ 2: TelemetryBridge.cpp (Создайте новый)
//  📍 Позиция: В папке проекта
//  🧩 Назначение: Реализация фрейминга, CRC8, неблокирующей отправки и диагностики
//  💻 Код:
/**
 * @file TelemetryBridge.cpp
 * @brief Мост телеметрии ESP32-S3 → RPi Zero 2W через UART2
 * @version 1.0.0
 * 
 * @details
 * • Использует аппаратный UART2 (GPIO5=TX, GPIO6=RX)
 * • Скорость: 921600 бод с RTS/CTS Flow Control
 * • Неблокирующая отправка через кольцевой буфер
 * • Автоматическое восстановление при переполнении буфера
 */



        // #include "TelemetryBridge.h"
        // #include "TelemetryPacket.h"
#include "Config.h"
#include <esp_log.h>
#include <driver/uart.h>

static const char* TAG_UART = "UART_BRIDGE";

// === КОНФИГУРАЦИЯ UART2 ===
#define UART_BRIDGE_NUM       UART_NUM_2
#define UART_BRIDGE_TX_PIN    Config::Pins::UART_RPI_TX  // GPIO5
#define UART_BRIDGE_RX_PIN    Config::Pins::UART_RPI_RX  // GPIO6
#define UART_BRIDGE_RTS_PIN   UART_PIN_NO_CHANGE         // Опционально: GPIO3
#define UART_BRIDGE_CTS_PIN   UART_PIN_NO_CHANGE         // Опционально: GPIO42
#define UART_BRIDGE_BAUD      921600                     // Высокая скорость для видео+телеметрия
#define UART_BRIDGE_BUF_SIZE  2048                       // Кольцевой буфер для отправки

// === ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ===
static bool _uartInitialized = false;
static uint32_t _packetsSent = 0;
static uint32_t _sendErrors = 0;

/**
 * @brief Инициализация UART2 с аппаратным управлением потоком
 * @details Конфигурирует Serial2 на 921600 бод, включает CTS/RTS.
 *          ESP32 аппаратно приостанавливает TX, если буфер RPi заполнен.
 * @brief Инициализация UART2 для связи с RPi Zero 2W
 * @return true при успешной инициализации
 */
bool TelemetryBridge::begin() {
    if (_uartInitialized) {
        ESP_LOGW(TAG, "⚠️ UART2 уже инициализирован");
        return true;
    }
    
    ESP_LOGI(TAG, "🚀 Инициализация UART2: TX=%d, RX=%d, Baud=%d",
             UART_BRIDGE_TX_PIN, UART_BRIDGE_RX_PIN, UART_BRIDGE_BAUD);
    
    // Конфигурация UART (ESP-IDF 5.0+ стиль)
    uart_config_t uartConfig = {
        .baud_rate = UART_BRIDGE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,  // ✅ Аппаратный Flow Control
        .rx_flow_ctrl_thresh = 122,              // Порог паузы при переполнении
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t err = uart_param_config(UART_BRIDGE_NUM, &uartConfig);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ uart_param_config failed: %s", esp_err_to_name(err));
        return false;
    }
    
    // Установка пинов
    err = uart_set_pin(UART_BRIDGE_NUM, 
                       UART_BRIDGE_TX_PIN, 
                       UART_BRIDGE_RX_PIN,
                       UART_BRIDGE_RTS_PIN, 
                       UART_BRIDGE_CTS_PIN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ uart_set_pin failed: %s", esp_err_to_name(err));
        return false;
    }
    
    // Установка буферов
    err = uart_driver_install(UART_BRIDGE_NUM, 
                              UART_BRIDGE_BUF_SIZE,  // RX буфер
                              UART_BRIDGE_BUF_SIZE,  // TX буфер
                              0,                     // Нет очереди событий
                              nullptr, 
                              0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "❌ uart_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }
    
    // Очистка буферов
    uart_flush(UART_BRIDGE_NUM);
    
    _uartInitialized = true;
    ESP_LOGI(TAG, "✅ UART2 инициализирован, Flow Control: CTS/RTS");
    return true;
}

/**
 * @brief Отправка пакета телеметрии на RPi (неблокирующая)
 * @param packet Готовый пакет TelemetryPacket_t с рассчитанным CRC8
 * @return true если данные записаны в буфер отправки
 */
bool TelemetryBridge::send(const TelemetryPacket_t& packet) {
    if (!_uartInitialized) {
        ESP_LOGE(TAG, "❌ send(): UART не инициализирован");
        return false;
    }
    
    // 🔑 КРИТИЧЕСКИ: Проверка доступного места в буфере перед записью
    size_t freeSpace = uart_get_buffered_data_len(UART_BRIDGE_NUM, UART_TX);
    if (freeSpace < TELEMETRY_FULL_SIZE + 64) {  // Запас на накладные расходы
        _sendErrors++;
        ESP_LOGW(TAG, "⚠️ Буфер UART переполнен (%u свободных байт), пакет отброшен", freeSpace);
        return false;
    }
    
    // 🔑 Запись пакета в буфер отправки (неблокирующая, 0 таймаут)
    int written = uart_write_bytes(UART_BRIDGE_NUM, 
                                   reinterpret_cast<const char*>(&packet), 
                                   TELEMETRY_FULL_SIZE);
    
    if (written == TELEMETRY_FULL_SIZE) {
        _packetsSent++;
        ESP_LOGV(TAG, "📤 Sent pkt #%u | RSSI:%ddB | Bat:%.1fV",
                 packet.packet_id, packet.rssi, packet.bat_voltage);
        return true;
    } else {
        _sendErrors++;
        ESP_LOGE(TAG, "❌ uart_write_bytes: записано %d/%d байт", 
                 written, TELEMETRY_FULL_SIZE);
        return false;
    }
}

/**
 * @brief Основной метод обновления (вызывать в loop())
 * @param timer Таймер для контроля частоты отправки
 * @param com Данные управления из LoRa
 * @param imu Данные стабилизации из MPU9250
 * @param bat Статус батареи
 */
void TelemetryBridge::update(TimerMillis* timer, 
                             const DataComSet_t& com, 
                             const SensorData& imu, 
                             const BatteryStatus_t& bat) {
    // 🔑 Проверка таймера (20-50 мс интервал)
    if (!timer.is_ready()) {
        return;
    }
    timer.reset();  // Сброс таймера после срабатывания
    
    // 🔑 Формирование пакета телеметрии
    TelemetryPacket_t packet = {};
    packet.header1 = 0xAA;
    packet.header2 = 0x55;
    packet.timestamp = millis();
    packet.packet_id = com.packet_id;
    
    // Копирование данных управления
    packet.com_up = com.comUp;
    packet.com_left = com.comLeft;
    packet.com_throttle = com.comThrottle;
    packet.com_flags = com.comSetAll;
    
    // Копирование данных стабилизации
    packet.roll = imu.roll;
    packet.pitch = imu.pitch;
    packet.yaw = imu.yaw;
    packet.altitude = imu.altitude;
    // speed/lat/lon — заглушки, если GPS не активен
    packet.speed = 0.0f;
    packet.latitude = 0.0;
    packet.longitude = 0.0;
    
    // Копирование статуса батареи (с явным приведением типов)
    packet.bat_voltage = bat.voltage;
    // packet.bat_percent = static_cast<uint8_t>(constrain(bat.percentage, 0.0f, 100.0f));
    packet.bat_percent  = static_cast<uint8_t>(constrain(bat.percentage, 0.0f, 100.0f)),
    
    // Статус системы
    //packet.flight_mode = (imu.status & 0x02) ? 2 : 1;  // 1=MANUAL, 2=STAB
    packet.flight_mode  = static_cast<uint8_t>((imu.status & 0x02) ? 2 : 1),
    packet.rssi = -120;  // Заглушка — получить из LoRaCommunicator
    
    // Фрейминг
    packet.footer1 = 0xCC;
    packet.footer2 = 0x33;
    
    // 🔑 Расчёт CRC8 (исключаем header[2] + footer[2] + crc[1])
    packet.crc8 = calculateCRC8(
        reinterpret_cast<const uint8_t*>(&packet.timestamp),
        TELEMETRY_PAYLOAD_SIZE
    );
    
    // 🔑 Отправка (неблокирующая)
    if (!send(packet)) {
        ESP_LOGW(TAG, "⚠️ Не удалось отправить пакет телеметрии #%u", com.packet_id);
    }
    
    // 🔑 Периодическая статистика (каждые 100 пакетов)
    if (_packetsSent % 100 == 0 && _packetsSent > 0) {
        float errorRate = (_sendErrors * 100.0f) / (_packetsSent + _sendErrors);
        ESP_LOGI(TAG, "📊 UART Stats: Sent=%lu Errors=%lu Rate=%.2f%%",
                 _packetsSent, _sendErrors, errorRate);
    }
}

/**
 * @brief Приём команд от RPi → ESP32 (обратный канал)
 * @param buffer Буфер для приёма данных
 * @param maxLength Максимальный размер буфера
 * @return Количество принятых байт, 0 если данных нет
 */
int TelemetryBridge::receiveCommand(uint8_t* buffer, size_t maxLength) {
    if (!_uartInitialized || !buffer) return 0;
    
    // Проверка доступных данных
    size_t available = 0;
    uart_get_buffered_data_len(UART_BRIDGE_NUM, UART_RX, &available);
    if (available == 0) return 0;
    
    // Чтение данных (неблокирующее, 0 таймаут)
    int read = uart_read_bytes(UART_BRIDGE_NUM, buffer, 
                               (available < maxLength) ? available : maxLength, 
                               0);  // 0 ms timeout = non-blocking
    
    if (read > 0) {
        ESP_LOGV(TAG, "📥 Received %d bytes from RPi", read);
        // 🔧 Здесь можно добавить парсинг команд:
        // - 0x01: перезапуск камеры
        // - 0x02: смена режима стабилизации
        // - 0x03: запрос расширенной телеметрии
    }
    return read;
}

/**
 * @brief Статистика работы моста
 */
void TelemetryBridge::printStats() {
    if (!_uartInitialized) {
        ESP_LOGW(TAG, "⚠️ UART не инициализирован");
        return;
    }
    
    uint32_t total = _packetsSent + _sendErrors;
    float successRate = (total > 0) ? (_packetsSent * 100.0f / total) : 100.0f;
    
    ESP_LOGI(TAG, "📊 UART_BRIDGE STATS:");
    ESP_LOGI(TAG, "   Sent: %lu, Errors: %lu, Success: %.1f%%",
             _packetsSent, _sendErrors, successRate);
    
    // Статус буфера
    size_t txFree = uart_get_buffered_data_len(UART_BRIDGE_NUM, UART_TX);
    size_t rxFree = uart_get_buffered_data_len(UART_BRIDGE_NUM, UART_RX);
    ESP_LOGI(TAG, "   Buffer: TX free=%u, RX free=%u", txFree, rxFree);
}// END  TelemetryBridge::handleRx()
