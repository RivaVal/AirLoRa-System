


// Здесь полностью переписана логика инициализации и отрисовки под синтаксис `GyverOLED`.
// *Обрати внимание:* Для дисплея 128x32 координата Y=32 — это уже край. 
// Мы разместим текст компактно.
//

/**
 * @file DisplayManager.cpp
 * @brief Реализация менеджера дисплея на базе GyverOLED
 * 
 * Особенности реализации:
 * - Используется режим буфера (OLED_BUFFER) для устранения мерцания.
 * - Явная очистка буфера перед каждым кадром.
 * - Подробное логирование каждого шага через ESP_LOG.
 */

/**
 * @file DisplayManager.cpp
 * @brief Реализация менеджера дисплея на базе GyverOLED
 * 
 * ИСПРАВЛЕНИЯ:
 * - Убрана проверка возврата init() (он void).
 * - Заменен printHEX на printf.
 * - Заменен drawRect на rect (метод GyverOLED).
 * - Добавлена явная инициализация Wire перед init().
 */

#include "DisplayManager.h"
#include <esp_log.h>
#include <Wire.h>

static const char* TAG = "DISPLAY";
// const char* TAG = "DISPLAY";

DisplayManager::DisplayManager()
    : _isInitialized(false),
      _lastUpdateMillis(0),
      _startTime (0)
{
    ESP_LOGI(TAG, "Constructor: GyverOLED object created (128x64, BUFFER mode)");
}

bool DisplayManager::begin() {
    ESP_LOGI(TAG, "=== START GYVER OLED INIT ===");

    // Установить время  старта программы
    _startTime = millis();    

    // 1. Лог конфигурации
    ESP_LOGI(TAG, "Config: SDA=%d, SCL=%d, Addr=0x%X", 
             Config::Pins::DISPLAY_SDA_PIN, 
             Config::Pins::DISPLAY_SCL_PIN, 
             Config::Pins::DISPLAY_I2C_ADDRESS);

    // 2. Явная инициализация шины Wire (критично для GyverOLED)
    // GyverOLED сам инициализирует Wire, поэтому можно убрать 
    // дублирующий вызов в `DisplayManager.cpp`:
    Wire.begin(Config::Pins::DISPLAY_SDA_PIN, Config::Pins::DISPLAY_SCL_PIN);
    delay(10); 

    // 3. Инициализация дисплея
    // В GyverOLED метод init() возвращает void. 
    // Мы просто вызываем его. Если дисплей не ответит, библиотека может зависнуть или молчать,
    // но обычно она пытается опросить шину.
    ESP_LOGI(TAG, "Calling GyverOLED init...");
    
    // Попытка инициализации с адресом из конфига
    _display.init(Config::Pins::DISPLAY_I2C_ADDRESS);
    // Небольшая задержка после инициализации
    delay(100);

    //-----------------------------------------------------------
    // Проверка связи с дисплеем через I2C
    Wire.beginTransmission(Config::Pins::DISPLAY_I2C_ADDRESS);
    int i2cError = Wire.endTransmission();
    if (i2cError != 0) {
        ESP_LOGE(TAG, "❌ I2C device not responding (error: %d)", i2cError);
        _isInitialized = false;
        return false;
    }
    ESP_LOGI(TAG, "✅ I2C device confirmed at 0x%X", Config::Pins::DISPLAY_I2C_ADDRESS);
    //-----------------------------------------------------------
    // Считаем успешным, если дошли сюда (GyverOLED не кидает исключений)
    _isInitialized = true;
    ESP_LOGI(TAG, "✅ Init call completed. Assuming success.");

    // 4. ТЕСТОВАЯ ОТРИСОВКА (Self-Test)
    ESP_LOGI(TAG, "Running self-test drawing...");
    _display.setContrast(255);           // Максимальная яркость
    _display.invertDisplay(false);       // ❗ Ключевое: отключаем инверсию
    _display.clear();                    // Очищаем буфер после смены режима
    _display.update();                   // Применяем изменения
    
    _display.clear();           // Очистить буфер
    _display.setScale(1);       // Шрифт 6x8
   _display.setCursor(0, 0);
    _display.print("GYVER OK");
    _display.setCursor(0, 10);
    _display.print("Addr:0x");
    _display.printf("%X", Config::Pins::DISPLAY_I2C_ADDRESS);

    ESP_LOGI(TAG, "Drawing test frame: %dx%d", 
            Config::Pins::DISPLAY_WIDTH, 
            Config::Pins::DISPLAY_HEIGHT);
    _display.rect(
        0,
        0, 
        Config::Pins::DISPLAY_WIDTH - 1, 
        Config::Pins::DISPLAY_HEIGHT - 1);   // ← Используем конфиг!
    _display.update();

    ESP_LOGI(TAG, "Test image sent. Wait 2s...");
    delay(2000);

    // ✅ ИСПРАВЛЕНО: ОДНА очистка вместо двух
    // [DISPLAY] begin() - финальная подготовка буфера
    //  [ 3680][I][DisplayManager.cpp:108] begin(): [DISPLAY] Clearing buffer after self-test
    _display.clear();
    _display.update();

    // [DISPLAY] begin() - завершение инициализации
    // [ 3686][I][DisplayManager.cpp:111] begin(): [DISPLAY] === DISPLAY INIT COMPLETE ===
    ESP_LOGI(TAG, "=== DISPLAY INIT COMPLETE ===");
    return true;
}


// 🔧 ИСПРАВЛЕННЫЙ update() для передачи реальных данных
void DisplayManager::update(const DataComSet_t& data) {
    if (!_isInitialized) return;
    if (millis() - _lastUpdateMillis >= Config::Display::UPDATE_INTERVAL_MS) {
        _lastUpdateMillis = millis();
        // Передаем актуальные значения вместо заглушек 90/90
        drawScreen(data.comUp, data.comLeft, data.comThrottle);
    }
}



void DisplayManager::forceRedraw(const DataComSet_t& data) {
    if (!_isInitialized) return;
    _lastUpdateMillis = millis();
    drawScreen(90, 90, data.comThrottle);
    ESP_LOGI(TAG, "Force redraw executed.");
}

// ============================================================================
// 🔧 ИСПРАВЛЕННАЯ ОТРИСОВКА (128x64 SSH1106/SSD1306)
// ============================================================================
/**
 * @brief Отрисовка данных на дисплее
 * @note Явное позиционирование, очистка буфера, вывод 4 строк
 */
void DisplayManager::drawScreen(uint8_t comUp, uint8_t comLeft, uint16_t comThrottle) {
    if (!_isInitialized) return;

    ESP_LOGD(TAG, "Рендеринг экрана: U=%u L=%u T=%u", comUp, comLeft, comThrottle);

    _display.clear();              // 1. Очистка буфера
    _display.setScale(1);          // 2. Шрифт 6x8

    // Заголовок
    _display.setCursor(0, 0);
    _display.print("TX DATA ");     // Заголовок

    // 🆕 ВРЕМЯ СО СТАРТА ПРОГРАММЫ (формат MM:SS)
    // 🔑 Используем uint32_t для секунд, uint8_t для отображаемых мин/сек
    uint32_t uptime_sec = ( millis() - _startTime ) / 1000;   // время в сек, Перевод в сек
    uint8_t mins = uptime_sec / 60;  // время в мин, перевод в мин
    uint8_t secs = uptime_sec % 60;  // время с точностью до сек

    //_display.setCursor(0, 48);
    _display.print("T: ");
    // 🔑 ИСПРАВЛЕНО: корректный нуль-паддинг для двузначных чисел
    if (mins < 10) _display.print('0');
    _display.print(mins);
    _display.print(':');
    if (secs < 10) _display.print('0');
    _display.print(secs);

    // 3. Данные с отступами по Y (12px шаг)
    _display.setCursor(0, 1);
    _display.print("U: "); _display.print(comUp);

    _display.setCursor(0, 2);
    _display.print("L: "); _display.print(comLeft);

    _display.setCursor(0, 3);
    _display.print("T: "); _display.print(comThrottle);


    _display.setCursor(0, 4);
    _display.print("ACK: ");  _display.print("00");

    _display.setCursor(0, 5);
    for (uint8_t i = 0; i < 21; i++) _display.print('='); // 21 * 6px = 126px

    _display.setCursor(0, 6);
    _display.print("OK");

    _display.update();             // 🔑 КРИТИЧНО: Отправка буфера на экран
    ESP_LOGD(TAG, "✅ Экран обновлен");
}




/*
// ✅ ИСПРАВЛЕННАЯ ВЕРСИЯ drawScreen() для 128×64:
// [DISPLAY] drawScreen() - отрисовка данных на экране
void DisplayManager::drawScreen(uint8_t comUp, uint8_t comLeft, uint16_t comThrottle) {
    if (!_isInitialized) {
        ESP_LOGW(TAG, "Draw skipped: Not initialized");
        return;
    }

    // [ 3820][D][DisplayManager.cpp:155] drawScreen(): [DISPLAY] Rendering: U=%d L=%d T=%d
    ESP_LOGD(TAG, "Rendering: U=%d L=%d T=%d", comUp, comLeft, comThrottle);

    // 1. Полная очистка буфера
    _display.clear();

    // 2. Настройки шрифта
    _display.setScale(1); // Шрифт 6x8 пикселей
    _display.setCursor(0, 0);

    // 3. Рисуем рамку (ИСПРАВЛЕНО: rect вместо drawRect)
    // rect(x, y, width, height) — ширина/высота, а не координаты!
    // _display.rect(0, 0, 127, 31);
    _display.rect(0, 0, 
            Config::Pins::DISPLAY_WIDTH - 1, 
            Config::Pins::DISPLAY_HEIGHT - 1);

    // 4. Заголовок
    _display.setCursor(0, 0);
    _display.print("TX DATA");
    // _display.print("TX DATA");

    // 5. Данные — координаты для высоты 64px (шаг ~12px)
    _display.setCursor(2, 12);   // Y=12
    _display.print("U:"); _display.print(comUp);

    // Строка 2: LEFT (Y=24)
    _display.setCursor(2, 24);
    _display.print("L:");       _display.print(comLeft);

    // Строка 3: THR (Y=36) - влезает впритык, так как высота 32
    _display.setCursor(2, 36);
    _display.print("T:");       _display.print(comThrottle);
    
    // Строка 4: Статус (Y=48) — дополнительная информация
    _display.setCursor(2, 48);
    _display.print("OK");
        // [ 3820][D][DisplayManager.cpp:162] drawScreen(): [DISPLAY] Preparing update: U=%d L=%d T=%d H=%d
        ESP_LOGD(TAG, "Preparing update: U=%d L=%d T=%d H=%d", 
                comUp, comLeft, comThrottle, Config::Pins::DISPLAY_HEIGHT);
    
    // 6. ОТПРАВКА БУФЕРА НА ЭКРАН
    _display.update();

    // [DISPLAY] drawScreen() - подтверждение отправки
    //[ 3821][D][DisplayManager.cpp:165] drawScreen(): [DISPLAY] Buffer sent to display
    ESP_LOGD(TAG, "✅ Buffer sent to display");

    // Отладка
    // ESP_LOGD(TAG, "Buffer drawn & updated: U=%d L=%d T=%d", comUp, comLeft, comThrottle);
    ESP_LOGD(TAG, "Buffer drawn & updated: U=%d L=%d T=%d | H=%d", 
             comUp, comLeft, comThrottle, Config::Pins::DISPLAY_HEIGHT);

}
*/
