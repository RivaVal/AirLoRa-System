


// I2C_Master.h
#ifndef I2C_MASTER_H
#define I2C_MASTER_H

/**
 * @file I2C_Master.h
 * @brief Нативный контроллер I2C шины для ESP-IDF 5.0+ (driver/i2c.h)
 * @note Полная замена устаревшего Wire.h для максимальной производительности и надёжности
 */

#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include "Config.h" // Предполагаем, что содержит пины SDA/SCL

#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C порт */

//#define I2C_MASTER_FREQ_HZ 400000 /*!< Частота шины 400 кГц */
#define I2C_MASTER_FREQ_HZ 100000  // ← ИЗМЕНИТЬ ЗДЕСЬ! /*!< Частота шины 100 кГц */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< Отключить буфер передачи */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< Отключить буфер приёма */
#define I2C_MASTER_TIMEOUT_MS 1000 /*!< Таймаут операций */

class I2CMasterController {
public:
    /**
     * @brief Инициализация I2C шины
     * @return true при успешной инициализации
     */
    bool begin();

    /**
     * @brief Сканирование устройств на шине I2C
     * @note Выводит список всех обнаруженных устройств через ESP_LOG
     */
    void scanDevices();

    /**
     * @brief Проверка подключения устройства по адресу
     * @param address I2C адрес устройства (7-bit)
     * @return true если устройство отвечает
     */
    bool isDeviceConnected(uint8_t address);

    /**
     * @brief Чтение данных из регистра устройства
     * @param dev_addr Адрес устройства
     * @param reg_addr Адрес регистра
     * @param data Буфер для чтения
     * @param len Количество байт для чтения
     * @return true при успехе
     */
    bool readRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len);

    /**
     * @brief Запись данных в регистр устройства
     * @param dev_addr Адрес устройства
     * @param reg_addr Адрес регистра
     * @param data Буфер с данными
     * @param len Количество байт для записи
     * @return true при успехе
     */
    bool writeRegister(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, size_t len);

    /**
     * @brief Универсальная передача данных (например, для burst read/write)
     * @param dev_addr Адрес устройства
     * @param write_buffer Буфер для записи (может быть nullptr)
     * @param write_size Размер буфера записи
     * @param read_buffer Буфер для чтения (может быть nullptr)
     * @param read_size Размер буфера чтения
     * @return true при успехе
     */
    bool transfer(uint8_t dev_addr, const uint8_t* write_buffer, size_t write_size, uint8_t* read_buffer, size_t read_size);

    /**
     * @brief Проверка, была ли инициализация успешной
     * @return true если инициализирован
     */
    bool isInitialized() const { return _initialized; }

private:
    static const char* TAG;
    bool _initialized = false;

    /**
     * @brief Внутренняя функция для выполнения I2C команды
     * @param cmd_handle Обработчик команды
     * @return esp_err_t результат выполнения
     */
    esp_err_t _i2c_master_cmd_begin(i2c_cmd_handle_t cmd_handle);
};

#endif // I2C_MASTER_H
