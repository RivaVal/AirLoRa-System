


// Вот полный пересмотренный код `DisplayManager`, адаптированный под `GyverOLED`.
// ### 1. Файл: `DisplayManager.h`
// Здесь мы меняем заголовки и тип объекта дисплея.
//

/**
 * @file DisplayManager.h
 * @brief Менеджер для управления I2C OLED дисплеем на базе GyverOLED
 * 
 * ИЗМЕНЕНИЯ:
 * - Переход на библиотеку GyverOLED
 * - Размер дисплея: 128x32 пикселя
 * - Режим работы: OLED_BUFFER (для стабильной отрисовки)
 */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
// Подключаем библиотеку GyverOLED
#include <GyverOLED.h> 

#include "Config.h"
#include "CommonTypes.h"

// ✅ ВОЗМОЖНОЕ ИСПРАВЛЕНИЕ (если дисплей SH1106):
// [DISPLAY] DisplayManager.h - тип контроллера
// GyverOLED<SSH1106_128x64, OLED_BUFFER> _display;


class DisplayManager {
public:
    DisplayManager();
    bool begin();
    void update(const DataComSet_t& data);
    void forceRedraw(const DataComSet_t& data);

private:
    // ВАЖНО: Указываем тип дисплея и режим буфера
    // SSD1306_128x32 - тип матрицы
    // OLED_BUFFER - режим работы (буферизированный)
    // GyverOLED<SSD1306_128x64, OLED_BUFFER> _display;
    // GyverOLED<SSD1306_128x64> _display;
    // ✅ ИСПРАВЛЕНИЕ (если дисплей SH1106):
    // [DISPLAY] DisplayManager.h - тип контроллера дисплея
    // Исправлено: SSD1306 → SSH1106 для совместимости с вашим дисплеем
    // Маркировка дисплея: < 1.30"IIC > обычно указывает на SH1106
    // [ 0][I][DisplayManager.h:45] class DisplayManager: [DISPLAY] Using SSH1106_128x64 controller
    GyverOLED<SSH1106_128x64, OLED_BUFFER> _display;
    
    bool _isInitialized;
    uint32_t _lastUpdateMillis;
    uint32_t _startTime ;

    void drawScreen(uint8_t comUp, uint8_t comLeft, uint16_t comThrottle);
    //static const char* TAG;
};

#endif // DISPLAY_MANAGER_H
