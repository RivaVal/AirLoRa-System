

// I2C_Master.cpp
#include "I2C_Master.h"
#include <cstring> // for memset

// static const char* TAG_FLIGHT = "FLIGHT_STAB";  // ← ДОБАВИТЬ
// static const char* I2CMasterController::TAG_I2C = "I2C_MASTER";
static const char* TAG_I2C = "I2C_MASTER";

/**
 * @brief Инициализация I2C Master (совместимо с ESP32-S3)
 * @details Исправлен флаг clk_flags (убрана опечатка FOR_NOMAL).
 *          Добавлена диагностика частоты и состояния шины.
 */
bool I2CMasterController::begin() {
    ESP_LOGI(TAG_I2C, "=== ИНИЦИАЛИЗАЦИЯ НАТИВНОГО I2C MASTER (ESP-IDF 5.0+) ===");
    ESP_LOGI(TAG_I2C, "Параметры: SDA=GPIO%d, SCL=GPIO%d, частота=%d Гц, порт=%d",
             Config::I2C::SDA_PIN, Config::I2C::SCL_PIN, I2C_MASTER_FREQ_HZ, I2C_MASTER_NUM);

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)Config::Pins::I2C_SDA;  // ← Было: Config::I2C::SDA_PIN
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)Config::Pins::I2C_SCL;  // ← Было: Config::I2C::SCL_PIN
    conf.master.clk_speed = Config::I2C::FREQUENCY;        // ← Частота из Config::I2C
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    /**
    * @brief Инициализация I2C Master (совместимо с ESP-IDF 5.0+/S3)
    * @details clk_flags=0 включает авто-выбор источника (APB_CLK или XTAL),
    *          что устраняет ошибки компиляции в IDF 5.1+
    */
    conf.clk_flags = 0;  // Авто-выбор тактирования (совместимо с IDF 5.1+)

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_I2C, "❌ Ошибка конфигурации I2C: %s", esp_err_to_name(err));
        return false;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
        I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_I2C, "❌ Ошибка установки драйвера I2C: %s", 
            esp_err_to_name(err));
        return false;
    }

    _initialized = true;
    ESP_LOGI(TAG_I2C, "✅ I2C Master инициализирован успешно.");
    return true;
}

void I2CMasterController::scanDevices() {
    if (!_initialized) {
        // ESP_LOGE(TAG_I2C, "❌ Сканирование невозможно: I2C не инициализирован");
        ESP_LOGI(TAG, "✅ I2C Master инициализирован: SDA=%d, SCL=%d, Freq=%d Гц",
                Config::I2C::SDA_PIN, Config::I2C::SCL_PIN, I2C_MASTER_FREQ_HZ);
        return;
    }
    ESP_LOGI(TAG_I2C, "=== СКАНИРОВАНИЕ I2C ШИНЫ (адреса 0x08-0x77) ===");
    uint8_t foundCount = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (isDeviceConnected(addr)) {
            const char* deviceName = "Unknown";
            switch(addr) {
                case 0x40: deviceName = "PCA9685 (PWM driver)"; break;
                case 0x68: deviceName = "MPU6050/MPU9250 (IMU)"; break;
                case 0x1E: deviceName = "HMC5883L (Magnetometer)"; break;
                case 0x77: deviceName = "BMP180/BMP280 (Barometer)"; break;
                default: break;
            }
            // ✅  Используем deviceName в выводе
            // ESP_LOGI(TAG_I2C, "✅ Устройство найдено: 0x%02X (%s)", addr, deviceName);
            // ✅ ИСПОЛЬЗОВАТЬ deviceName в выводе:
            ESP_LOGI(TAG_I2C, "✅ Устройство: 0x%02X (%s)", addr, deviceName);  // ← Использовать обе переменные
            foundCount++;
        }
        // Small delay for stability during scan
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    if (foundCount == 0) {
        ESP_LOGW(TAG_I2C, "⚠️ Предупреждение: На шине I2C не обнаружено устройств");
    } else {
        ESP_LOGI(TAG_I2C, "✅ Обнаружено %d устройств(а) на шине I2C", foundCount);
    }
    
}

// === ДОБАВИТЬ в isDeviceConnected() для лучшей диагностики ===
bool I2CMasterController::isDeviceConnected(uint8_t address) {
    if (!_initialized) {
        ESP_LOGE(TAG_I2C, "❌ Проверка подключения невозможна: I2C не инициализирован");
        return false;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        ESP_LOGD(TAG_I2C, "Устройство 0x%02X отвечает", address);
        return true;
    } else {
        ESP_LOGD(TAG_I2C, "Устройство 0x%02X не отвечает (код ошибки: %s)",
                 address, esp_err_to_name(err));
        return false;
    }
}

bool I2CMasterController::readRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t len) {
    if (!_initialized) {
        ESP_LOGE(TAG_I2C, "❌ Чтение регистра невозможно: I2C не инициализирован");
        return false;
    }
    ESP_LOGV(TAG_I2C, "Чтение регистра: устройство=0x%02X, регистр=0x%02X, длина=%u",
             dev_addr, reg_addr, len);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd); // Повторный старт
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = _i2c_master_cmd_begin(cmd);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG_I2C, "❌ Ошибка чтения регистра 0x%02X устройства 0x%02X: %s",
                 reg_addr, dev_addr, esp_err_to_name(err));
        return false;
    }
    ESP_LOGV(TAG_I2C, "Чтение успешно: данные=[");
    for (size_t i = 0; i < len; i++) {
        ESP_LOGV(TAG_I2C, "0x%02X%s", data[i], (i < len - 1) ? " " : "");
    }
    ESP_LOGV(TAG_I2C, "]");
    return true;
}

bool I2CMasterController::writeRegister(uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, size_t len) {
    if (!_initialized) {
        ESP_LOGE(TAG_I2C, "❌ Запись регистра невозможна: I2C не инициализирован");
        return false;
    }
    ESP_LOGV(TAG_I2C, "Запись регистра: устройство=0x%02X, регистр=0x%02X, длина=%u",
             dev_addr, reg_addr, len);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    for (size_t i = 0; i < len; i++) {
        i2c_master_write_byte(cmd, data[i], true);
    }
    i2c_master_stop(cmd);

    esp_err_t err = _i2c_master_cmd_begin(cmd);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG_I2C, "❌ Ошибка записи в регистр 0x%02X устройства 0x%02X: %s",
                 reg_addr, dev_addr, esp_err_to_name(err));
        return false;
    }
    ESP_LOGV(TAG_I2C, "Запись успешна");
    return true;
}

bool I2CMasterController::transfer(uint8_t dev_addr, const uint8_t* write_buffer, size_t write_size, uint8_t* read_buffer, size_t read_size) {
    if (!_initialized) {
        ESP_LOGE(TAG_I2C, "❌ Передача данных невозможна: I2C не инициализирован");
        return false;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    if (write_size > 0) {
        i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
        for (size_t i = 0; i < write_size; i++) {
            i2c_master_write_byte(cmd, write_buffer[i], true);
        }
    }
    if (read_size > 0) {
        if (write_size > 0) { // Если была запись, нужен повторный старт
            i2c_master_start(cmd);
        }
        i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
        if (read_size > 1) {
            i2c_master_read(cmd, read_buffer, read_size - 1, I2C_MASTER_ACK);
        }
        if (read_size > 0) { // Always read the last byte
            i2c_master_read_byte(cmd, read_buffer + read_size - 1, I2C_MASTER_NACK);
        }
    }
    i2c_master_stop(cmd);

    esp_err_t err = _i2c_master_cmd_begin(cmd);
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG_I2C, "❌ Ошибка передачи данных с устройством 0x%02X: %s",
                 dev_addr, esp_err_to_name(err));
        return false;
    }
    return true;
}

esp_err_t I2CMasterController::_i2c_master_cmd_begin(i2c_cmd_handle_t cmd_handle) {
    return i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}