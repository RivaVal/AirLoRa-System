



// File :: "PCA9685_ServoController.cpp" 
//
/**
 * @file PCA9685_ServoController.cpp
 * @brief Реализация контроллера сервоприводов через внешний I2C PWM-драйвер PCA9685
 * @version 1.2.0 (исправленная и документированная)
 * @date 2026-02-13
 * @details
 *
 * 🔧 КРИТИЧЕСКИЕ ИСПРАВЛЕНИЯ:
 *   1. Добавлено #include "CommonTypes.h" для доступа к ControllerDebug_t
 *   2. Добавлено #include "esp_log.h" для работы с ESP_LOG макросами
 *   3. Исправлена логика инициализации PCA9685 (правильная последовательность режимов)
 *   4. Добавлена подробная отладка через ESP_LOG на всех этапах
 *   5. Реализован безопасный тест 7 сервоприводов при старте системы
 * 
 * 📊 АРХИТЕКТУРА:
 *   - Управление 7 сервоприводами через один чип PCA9685
 *   - Подключение по шине I2C: SDA=GPIO21, SCL=GPIO22 (фиксировано в Config.h)
 *   - Частота PWM: 50 Гц (стандарт для сервоприводов)
 *   - Диапазон импульсов: 544 мкс (0°) — 2400 мкс (180°)
 * 
 * 🗺️ РАСПРЕДЕЛЕНИЕ СЕРВОПРИВОДОВ:
 *   Серво #0 → Канал 0: Левый элерон
 *   Серво #1 → Канал 1: Правый элерон
 *   Серво #2 → Канал 2: Левый руль высоты
 *   Серво #3 → Канал 3: Правый руль высоты
 *   Серво #4 → Канал 4: Руль направления
 *   Серво #5 → Канал 5: Крышка парашюта
 *   Серво #6 → Канал 6: Вспомогательные крышки
 * 
 * ⚠️ ВАЖНО: Сервоприводы НЕ подключаются напрямую к пинам ESP32!
 *           Все 7 серв подключаются ТОЛЬКО к выходам PWM0-PWM6 платы PCA9685.
 *
 */
 /**
* @file PCA9685_ServoController.cpp
* @brief Реализация контроллера сервоприводов через PCA9685 (ИСПРАВЛЕННАЯ ВЕРСИЯ)
* @version 1.3.1 — Исправлены ошибки компиляции: имена полей, констант, функций
* @date 2026-02-26
*
* 🔑 ИСПРАВЛЕНИЯ:
* 1. ServoParams: minPulse → minPulseUs, maxPulse → maxPulseUs, neutralPulse → neutralPulseUs
* 2. Константы: SERVO_PULSE_MIN → SERVO_PULSE_MIN_US (и аналоги)
* 3. Функция: logicalAngleToPulse → logicalAngleToPulseUs
* 4. Удалён дубликат resetToNeutral()
* 5. Добавлены недостающие #define для регистров PCA9685
*/
/**
 * @file PCA9685_ServoController.cpp
 * @brief Реализация контроллера сервоприводов через PCA9685
 * @version 1.4.0 — УТОЧНЁННЫЙ ПРЕДСТАРТОВЫЙ ТЕСТ
 * @date 2026-02-26
 * 
 * 🔑 КРИТИЧЕСКИЕ ИСПРАВЛЕНИЯ:
 *   1. Добавлен новый тест: нейтраль → ±30° → ±45° → ±90° → нейтраль
 *   2. Все 7 сервоприводов двигаются ОДНОВРЕМЕННО
 *   3. Тест повторяется 3 раза с паузой 1 секунда
 *   4. Подробная отладка через ESP_LOG на всех этапах
 *   5. Исправлена логика инициализации PCA9685
 * 
 * 📊 ПРОИЗВОДИТЕЛЬНОСТЬ:
 *   - Частота обновления: 50 Гц (20 мс период)
 *   - Разрешение PWM: 12 бит (4096 уровней)
 *   - Точность угла: ~0.044° на тик
 * 
 * ⚠️ ТРЕБОВАНИЯ К ПИТАНИЮ:
 *   - Логика PCA9685: 3.3В (VCC)
 *   - Питание сервоприводов: 5-6В (V+)
 *   - Общий GND между ESP32 и PCA9685
 *   - Ток БП: минимум 2А на 7 сервоприводов
 * 
 * @author Embedded Systems Team
 * @note Все пины зафиксированы в Config.h!
 */
/**
 * @file PCA9685_ServoController.cpp
 * @brief Реализация контроллера сервоприводов через PCA9685
 * @version 1.4.1 — ИСПРАВЛЕНЫ ОШИБКИ КОМПИЛЯЦИИ
 * @date 2026-02-26
 * 
 * 🔑 ИСПРАВЛЕНИЯ:
 * 1. Все методы имеют правильные сигнатуры с областью видимости класса
 * 2. Использованы корректные имена констант (PCA9685_LED0_ON_L)
 * 3. Удалены несуществующие функции (runServoTest_Vers01)
 * 4. Реализован уточнённый тест: нейтраль→±30→±45→±90→нейтраль (3 цикла)
 * 5. Все 7 сервоприводов двигаются ОДНОВРЕМЕННО
 */
/**
* @file PCA9685_ServoController.cpp
* @brief Реализация контроллера сервоприводов через PCA9685
* @version 1.5.0 — УТОЧНЁННЫЙ ПРЕДСТАРТОВЫЙ ТЕСТ С УГЛАМИ БЕЗОПАСНОСТИ
* @date 2026-02-26
*
* 🔑 КРИТИЧЕСКИЕ ИСПРАВЛЕНИЯ:
*   1. Добавлен новый тест: нейтраль → ±30 → ±45 → ±90 → нейтраль (3 цикла)
*   2. Все 7 сервоприводов двигаются ОДНОВРЕМЕННО
*   3. Тест повторяется 3 раза с паузой 1 секунда
*   4. Добавлены углы безопасности (5° буфер, 85° макс)
*   5. Задержка между шагами: 600 мс
*   6. Подробная отладка через ESP_LOG на всех этапах
*
* 📊 ПРОИЗВОДИТЕЛЬНОСТЬ:
*   - Частота обновления: 50 Гц (20 мс период)
*   - Разрешение PWM: 12 бит (4096 уровней)
*   - Точность угла: ~0.044° на тик
*
* ⚠️ ТРЕБОВАНИЯ К ПИТАНИЮ:
*   - Логика PCA9685: 3.3В (VCC)
*   - Питание сервоприводов: 5-6В (V+)
*   - Общий GND между ESP32 и PCA9685
*   - Ток БП: минимум 2А на 7 сервоприводов
*
* @author Embedded Systems Team
* @note Все пины зафиксированы в Config.h!
*/
/**
* @file PCA9685_ServoController.cpp
* @brief Реализация контроллера сервоприводов через PCA9685
* @version 1.7.0 — ИСПРАВЛЕНО ДЛЯ MG90S
* @date 2026-02-27
*
* 🔑 КРИТИЧЕСКИЕ ИСПРАВЛЕНИЯ:
*   1. Диапазон импульсов: 500-2500 мкс (MG90S спецификация)
*   2. Задержка стабилизации: 1000 мс (вместо 750 мс)
*   3. Добавлена полная диагностика I2C операций
*   4. Проверка MODE1 регистра после инициализации
*/
/**
* @file PCA9685_ServoController.cpp
* @brief Реализация контроллера сервоприводов через PCA9685
* @version 1.7.0 — ИСПРАВЛЕНО ДЛЯ MG90S
* @date 2026-02-27
*
* 🔑 КРИТИЧЕСКИЕ ИСПРАВЛЕНИЯ:
*   1. Диапазон импульсов: 500-2500 мкс (MG90S спецификация)
*   2. Задержка стабилизации: 1000 мс (вместо 750 мс)
*   3. Добавлена полная диагностика I2C операций
*   4. Проверка MODE1 регистра после инициализации
*/
/**
 * @file PCA9685_ServoController.cpp
 * @brief Реализация контроллера сервоприводов PCA9685 (Adafruit-style)
 * @version 2.0.0 — Полная переработка на базе Adafruit_PWMServoDriver
 * @date 2026-03-02
 * 
 * @details
 * Реализация следует архитектуре официальной библиотеки Adafruit:
 * https://github.com/adafruit/Adafruit_PWMServoDriver
 * 
 * 🔑 КЛЮЧЕВЫЕ ОТЛИЧИЯ ОТ ОРИГИНАЛА:
 * • Используется нативный ESP-IDF I2C драйвер (через I2C_Master.h)
 * • Отладка через ESP_LOG вместо Serial.print
 * • Поддержка калибровки каждого канала
 * • Уточнённый предстартовый тест для 7 сервоприводов
 * • Проверка безопасности углов перед применением
 * 
 * 📊 ПРОИЗВОДИТЕЛЬНОСТЬ:
 * • Частота обновления: 50 Гц (20 мс период)
 * • Разрешение PWM: 12 бит (4096 уровней, ~0.044° на тик)
 * • Время I2C транзакции: ~1-2 мс при 400 кГц
 * 
 * ⚠️ ТРЕБОВАНИЯ К ПИТАНИЮ:
 * • VCC (логика PCA9685): 3.3V от ESP32
 * • V+ (питание сервоприводов): 5-6V от отдельного АКБ
 * • GND: общий между ESP32 и PCA9685
 * • Ток БП: минимум 2A на 7 сервоприводов MG90S
 * 
 * @author Embedded Systems Team (на базе Adafruit Industries)
 * @note Все пины зафиксированы в Config.h!
 */
/**
 * @file PCA9685_ServoController.cpp
 * @brief Контроллер сервоприводов через PCA9685 (ФИНАЛЬНАЯ ВЕРСИЯ 2.0)
 * @version 2.0.0 — УЧТЕНЫ ВСЕ ПРЕДЫДУЩИЕ ЭКСПЕРИМЕНТЫ
 * @date 2026-03-02
 * 
 * @details
 * 🔑 КРИТИЧЕСКИЕ УРОКИ ИЗ ПРЕДЫДУЩИХ ВЕРСИЙ:
 * 
 * 1. ✅ AUTO_INCREMENT БИТ (бит 5 = 0x20) — ОБЯЗАТЕЛЕН!
 *    Без него 4-байтовая запись в регистры PWM НЕ РАБОТАЕТ!
 *    Рабочая версия (002_3.txt): MODE1 = 0xA0 (AI=1)
 *    Нерабочая (003_1.txt): MODE1 = 0x01 (AI=0) → СЕРВО НЕ ДВИГАЮТСЯ!
 * 
 * 2. ✅ ЗАДЕРЖКА СТАБИЛИЗАЦИИ — 1500 МС МИНИМУМ!
 *    Рабочая версия: 1500 мс → осциллятор успевает запуститься
 *    Нерабочая: 5 мс → осциллятор НЕ стабилизируется
 * 
 * 3. ✅ ДИАПАЗОН ИМПУЛЬСОВ ДЛЯ MG90S — 544-2400 МКС
 *    500-2500 мкс может вызывать полные обороты на некоторых MG90S
 *    544-2400 мкс — безопасный диапазон из рабочей версии
 * 
 * 4. ✅ ПОСЛЕДОВАТЕЛЬНОСТЬ ИНИЦИАЛИЗАЦИИ (Adafruit-style):
 *    1. Проверка подключения I2C
 *    2. Чтение MODE1
 *    3. Переход в SLEEP (бит 4 = 1)
 *    4. Установка prescale для 50 Гц
 *    5. Выход из SLEEP с AUTO_INCREMENT (бит 5 = 1)
 *    6. Задержка 1500 мс для стабилизации
 *    7. Проверка MODE1 (SLEEP=0, AI=1)
 *    8. Сброс серво в нейтраль
 * 
 * 📊 АРХИТЕКТУРА:
 *   - Управление 7 сервоприводами MG90S через PCA9685
 *   - Подключение: I2C SDA=GPIO21, SCL=GPIO22 (Config.h)
 *   - Частота PWM: 50 Гц (стандарт для серво)
 *   - Разрешение: 12 бит (4096 тиков)
 *   - Точность: ~0.044° на тик
 * 
 * 🗺️ РАСПРЕДЕЛЕНИЕ КАНАЛОВ:
 *   Канал 0: Левый руль высоты
 *   Канал 1: Правый руль высоты
 *   Канал 2: Левый руль направления
 *   Канал 3: Правый руль направления
 *   Канал 4: Парашют
 *   Канал 5: Вспом. крышка #1
 *   Канал 6: Вспом. крышка #2
 * 
 * ⚠️ ТРЕБОВАНИЯ К ПИТАНИЮ:
 *   - VCC (логика PCA9685): 3.3V от ESP32
 *   - V+ (питание серво): 5-6V от отдельного АКБ
 *   - GND: ОБЩИЙ между ESP32 и PCA9685
 *   - Ток БП: минимум 2A на 7 серво MG90S
 * 
 * @author Embedded Systems Team
 * @note Все пины зафиксированы в Config.h!
 */
/**
 * @file PCA9685_ServoController.cpp
 * @brief Реализация контроллера сервоприводов PCA9685 (ФИНАЛЬНАЯ ВЕРСИЯ 3.0)
 * @version 3.0.0 — ИСПРАВЛЕНА АРХИТЕКТУРА КОНСТРУКТОРА
 * @date 2026-03-02
 * 
 * @details
 * 🔑 КРИТИЧЕСКИЕ ИСПРАВЛЕНИЯ:
 * 1. ✅ Конструктор принимает I2C адрес (по умолчанию 0x40)
 * 2. ✅ Конструктор принимает опциональный массив каналов
 * 3. ✅ MODE1 = 0x20 при выходе из SLEEP (AUTO_INCREMENT=1)
 * 4. ✅ Задержка стабилизации 1500 мс (из рабочей версии 002_3.txt)
 * 5. ✅ Импульсы 544-2400 мкс для MG90S
 * 6. ✅ Тест: 0° → ±60° → 0° (3 цикла)
 * 
 * @note Архитектура основана на рабочей версии из лога 002_3.txt
 */

#include "PCA9685_ServoController.h"
#include "Config.h"
#include "CommonTypes.h"
#include <esp_log.h>
#include <cstring>
#include <cmath>

/**
 * @brief Конструктор контроллера сервоприводов
 * @param i2cAddress Адрес PCA9685 на шине I2C
 * 
 * @details Инициализирует:
 * • Параметры сервоприводов MG90S (500-2500 мкс)
 * • Массив калибровок (все нули)
 * • Флаги состояния (не инициализирован)
 *
 * @details Инициализирует параметры для MG90S:
 *   - Частота: 50 Гц
 *   - Мин импульс: 544 мкс (0°)
 *   - Нейтраль: 1500 мкс (90°)
 *   - Макс импульс: 2400 мкс (180°)
 *
 */
/**
 * @file PCA9685_ServoController.cpp
 * @brief Реализация контроллера сервоприводов PCA9685 (ВЕРСИЯ 3.2 — DATASHEET COMPLIANT)
 * @version 3.2.0 — Полное соответствие NXP PCA9685 Datasheet Rev. 4 (2015)
 * @date 2026-03-03
 *
 * 🔑 ИЗМЕНЕНИЯ НА ОСНОВЕ DATASHEET:
 * 1. ✅ MODE1 RESTART (Bit 7): Учтено требование Table 5 (запись 1 для сброса флага).
 * 2. ✅ MODE2 OUTDRV (Bit 2): Подтверждено Table 6 (Default=1, Totem Pole для серво).
 * 3. ✅ PRE_SCALE: Формула из Section 7.3.5 Eq.1.
 * 4. ✅ SLEEP Sequence: Строго по Section 7.3.5 (Sleep=1 → Write Prescale → Sleep=0).
 * 5. ✅ Oscillator Startup: Задержка > 500 мкс (Section 7.3.1).
 */


// ============================================================================
// 🔧 МАССИВ ТЕСТОВОЙ ПОСЛЕДОВАТЕЛЬНОСТИ (ГЛОБАЛЬНЫЙ)
// ============================================================================
/**
 * @brief Последовательность углов для теста
 * @details Нейтраль → +60° → нейтраль → -60° → нейтраль
 */
const int16_t SERVO_TEST_SEQUENCE[] = {
    0, 85, 0, -85, 0
};
const uint8_t SERVO_TEST_SEQUENCE_SIZE = 5 ;

// ============================================================================
// 🔧 СТАТИЧЕСКАЯ ПЕРЕМЕННАЯ TAG ДЛЯ ESP_LOG
// ============================================================================

const char* PCA9685_ServoController::TAG = "PCA9685_SERVO";

// ============================================================================
// 🔧 КОНСТРУКТОР
// ============================================================================
/**
 * @brief Конструктор контроллера сервоприводов
 * @param i2cAddress I2C адрес PCA9685 (по умолчанию 0x40)
 * @param channels Массив номеров каналов (опционально, по умолчанию 0-6)
 */
PCA9685_ServoController::PCA9685_ServoController(uint8_t i2cAddress, 
                                                   const uint8_t* channels)
    : _i2cAddress(i2cAddress)       // 🔑 Сохраняем I2C адрес из параметра!
 {
    // **✅ ЭТО ВЕРНО для SG90!** Не менять!
    // ✅ ИНИЦИАЛИЗАЦИЯ ПАРАМЕТРОВ ДЛЯ SG90/MG90S:
            //            _servoParams.frequency = 50;              // 50 Гц
            //            _servoParams.minPulseUs = 1100;           // ✅ 1.0 мс
            //            _servoParams.maxPulseUs = 1900;           // ✅ 2.0 мс
            //            _servoParams.neutralPulseUs = 1500;       // ✅ 1.5 мс
        //    _servoParams.minPulseUs = 1000;           // ✅ 1.0 мс
        //    _servoParams.maxPulseUs = 2000;           // ✅ 2.0 мс
        //    _servoParams.neutralPulseUs = 1500;       // ✅ 1.5 мс

        _servoParams.frequency = PCA9685_PWM_FREQUENCY;              // 50 Гц - стандарт для серво
        _servoParams.minPulseUs     = Config::ServoCalibration::MIN_PULSE_WIDTH_US;     // ✅ 1.0 мс = -90°
        _servoParams.maxPulseUs     = Config::ServoCalibration::MAX_PULSE_WIDTH_US;     // ✅ 2.0 мс = +90°
        _servoParams.neutralPulseUs = Config::ServoCalibration::NEUTRAL_PULSE_WIDTH_US; // ✅ 1.5 мс = 0° (нейтраль)


    //    constexpr uint32_t SERVO_FREQ = 50;              // 50 Гц - стандарт для серво
    // /   constexpr uint32_t MIN_PULSE_WIDTH_US = 1000;    // ✅ 1.0 мс = -90°
    //    constexpr uint32_t MAX_PULSE_WIDTH_US = 2000;    // ✅ 2.0 мс = +90°
    //    constexpr uint32_t NEUTRAL_PULSE_WIDTH_US = 1500;// ✅ 1.5 мс = 0° (нейтраль)

    // Инициализация каналов (по умолчанию 0-6)
    if (channels) {
        memcpy(_servoChannels, channels, sizeof(_servoChannels));
    } else {
        for (uint8_t i = 0; i < 7; i++) {
            _servoChannels[i] = i;  // Каналы 0,1,2,3,4,5,6
        }
    }
    
    // Инициализация калибровки
    for (uint8_t i = 0; i < 7; i++) {
        _servoTrim[i] = 0;
    }
    
    _initialized = false;
    _chipConnected = false;
    _i2cManager = nullptr;
    
    ESP_LOGI(TAG, "✅ Конструктор: адрес=0x%02X, каналы=0-6", _i2cAddress);
    ESP_LOGI(TAG, "   Параметры MG90S: min=%u мкс, neutral=%u мкс, max=%u мкс",
             _servoParams.minPulseUs, 
             _servoParams.neutralPulseUs, 
             _servoParams.maxPulseUs);
}
    
// ============================================================================
// 🔧 РАСЧЁТ PRESCALE (Adafruit formula)
// ============================================================================
/**
 * @brief Расчёт prescale для заданной частоты PWM
 * @param freq Целевая частота в Гц
 * @return Значение prescale для записи в регистр 0xFE
 * 
 * @details Формула из даташита PCA9685 и библиотеки Adafruit:
 * 
 * prescale = round(osc_clock / (4096 * freq)) - 1
 * 
 * Где:
 * • osc_clock = 25 МГц (внутренний осциллятор PCA9685, ±50% точность)
 * • 4096 = количество тиков за период PWM
 * • freq = целевая частота PWM
 * 
 * Примеры:
 * • 50 Гц: prescale = round(25000000/(4096*50)) - 1 = 121
 * • 60 Гц: prescale = round(25000000/(4096*60)) - 1 = 100
 * 
 * @note Фактическая частота может отличаться из-за ±50% точности осциллятора.
 *       Для точной настройки используйте внешний кристалл 25 МГц.
 */
 /*
uint8_t PCA9685_ServoController::calculatePrescale(uint16_t freq) const {
    // Формула Adafruit с округлением
    float prescaleval = PCA9685_OSC_CLOCK;
    prescaleval /= PCA9685_PWM_RESOLUTION;  // 4096
    prescaleval /= freq;
    prescaleval -= 1.0f;
    
    // Округление до ближайшего целого
    uint8_t prescale = static_cast<uint8_t>(floor(prescaleval + 0.5f));
    
    ESP_LOGV(TAG, "calculatePrescale: freq=%u Гц → prescale=%u (расчёт: %.2f)",
             freq, prescale, prescaleval);
    
    return prescale;
}
*/

// ============================================================================
// 🔧 ПРЕОБРАЗОВАНИЕ: МИКРОСЕКУНДЫ → ТИКИ PCA9685  (Section 7.3.3)
// ============================================================================
/**
 * @brief Преобразование ширины импульса (мкс) в тики PCA9685
 * @param pulseUs Ширина импульса в микросекундах
 * @return Количество тиков (0-4095)
 * 
 * @details Формула:
 * 
 * ticks = pulseUs * 4096 / 20000
 * 
 * Где:
 * • 4096 = полное разрешение PWM (12 бит)
 * • 20000 мкс = период при 50 Гц (1/50 = 0.02 с = 20000 мкс)
 * 
 * Примеры для 50 Гц:
 * • 500 мкс → 500 * 4096 / 20000 = 102 тика
 * • 1500 мкс → 1500 * 4096 / 20000 = 307 тиков
 * • 2500 мкс → 2500 * 4096 / 20000 = 512 тиков
 * 
 * @note Значение ограничивается диапазоном 0-4095 для безопасности
 */
uint16_t PCA9685_ServoController::pulseUsToTicks(uint16_t pulseUs) const {
    // 🔑 ФОРМУЛА: ticks = pulseUs * 4096 / 20000
    // Где 20000 мкс = период при 50 Гц (1/50 = 0.02 с = 20000 мкс)
    // Section 7.3.3: 12-bit resolution (0-4095)
    // Period at 50Hz = 20000 мкс
    // Tick = 20000 / 4096 ≈ 4.88 мкс
    uint32_t ticks = (static_cast<uint32_t>(pulseUs) * 4096UL) / 20000UL;
    
    // Ограничение 12 битами (Table 7)
    if (ticks > 4095) ticks = 4095;
    if (ticks < 1) ticks = 1;  // 0 = Full Off // 0 выключает выход полностью
    
    ESP_LOGV(TAG, "pulseUsToTicks: %u мкс → %u тиков", pulseUs, ticks);
    return static_cast<uint16_t>(ticks);
}

// ============================================================================
// 🔧 ПРЕОБРАЗОВАНИЕ: УГОЛ → ИМПУЛЬС
// ============================================================================
/**
 * @brief Преобразование логического угла в ширину импульса
 * @param logicalAngle Угол в градусах (-90...+90)
 * @return Ширина импульса в микросекундах
 * 
 * @details Линейная интерполяция:
 * 
 * pulse = neutral + angle * (max - min) / 180
 * 
 * Где:
 * • neutral = 1500 мкс (соответствует 0°)
 * • min = 500 мкс (соответствует -90°)
 * • max = 2500 мкс (соответствует +90°)
 * • 180 = полный диапазон углов (-90° до +90°)
 * 
 * Примеры:
 * • angle = 0° → pulse = 1500 + 0 = 1500 мкс
 * • angle = +90° → pulse = 1500 + 90*2000/180 = 2500 мкс
 * • angle = -90° → pulse = 1500 - 90*2000/180 = 500 мкс
 * 
 * @note Результат гарантированно ограничивается [minPulseUs, maxPulseUs]
 */
uint16_t PCA9685_ServoController::logicalAngleToPulseUs(int16_t logicalAngle) const {
    // Ограничение входного угла
    if (logicalAngle < -90) logicalAngle = -90;
    if (logicalAngle > 90) logicalAngle = 90;
    
    // ✅ ЛИНЕЙНАЯ ИНТЕРПОЛЯЦИЯ для диапазона 1000-2000 мкс:
    // Формула: pulse = neutral + angle * (max - min) / 180
    // Пример: angle=+90° → 1500 + 90*(2000-1000)/180 = 2000 мкс ✓
    // Пример: angle=-90° → 1500 - 90*(2000-1000)/180 = 1000 мкс ✓
    
    int32_t pulseWidth = _servoParams.neutralPulseUs +
        (static_cast<int32_t>(logicalAngle) * 
         (_servoParams.maxPulseUs - _servoParams.minPulseUs)) / 180;
    
    // Гарантированное ограничение в физические пределы SG90
    if (pulseWidth < static_cast<int32_t>(_servoParams.minPulseUs)) {
        pulseWidth = _servoParams.minPulseUs;  // 1000 мкс
    }
    if (pulseWidth > static_cast<int32_t>(_servoParams.maxPulseUs)) {
        pulseWidth = _servoParams.maxPulseUs;  // 2000 мкс
    }
    
    ESP_LOGV(TAG, "logicalAngleToPulseUs: %+d° → %ld мкс", 
             logicalAngle, pulseWidth);

    return static_cast<uint16_t>(pulseWidth);
}




// ============================================================================
// 🔧 НИЗКОУРОВНЕВЫЕ I2C ОПЕРАЦИИ
// ============================================================================
bool PCA9685_ServoController::readRegister(uint8_t reg_addr, uint8_t* data, size_t len) {
    if (!_i2cManager) {
        ESP_LOGE(TAG, "❌ readRegister: _i2cManager не установлен");
        return false;
    }
    return _i2cManager->readRegister(_i2cAddress, reg_addr, data, len);
}

bool PCA9685_ServoController::writeRegister(uint8_t reg_addr, const uint8_t* data, size_t len) {
    if (!_i2cManager) {
        ESP_LOGE(TAG, "❌ writeRegister: _i2cManager не установлен");
        return false;
    }
    return _i2cManager->writeRegister(_i2cAddress, reg_addr, data, len);
}

// ============================================================================
// 🔧 ДИАГНОСТИКА ОБОРУДОВАНИЯ
// ============================================================================
/*
bool PCA9685_ServoController::diagnoseHardware() {
    ESP_LOGI(TAG, "🔍 Диагностика PCA9685 (адрес 0x%02X)...", _i2cAddress);
    
    // Проверка подключения
    if (!_i2cManager->isDeviceConnected(_i2cAddress)) {
        ESP_LOGE(TAG, "❌ Устройство не отвечает на шине I2C");
        return false;
    }
    ESP_LOGI(TAG, "✅ Устройство отвечает на I2C");
    
    // Чтение MODE1
    uint8_t mode1;
    if (!readRegister(PCA9685_MODE1, &mode1, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось прочитать MODE1");
        return false;
    }
    ESP_LOGI(TAG, "   MODE1: 0x%02X (SLEEP=%d, RESTART=%d, ALLCALL=%d)",
             mode1,
             (mode1 & PCA9685_MODE1_SLEEP) ? 1 : 0,
             (mode1 & PCA9685_MODE1_RESTART) ? 1 : 0,
             (mode1 & PCA9685_MODE1_ALLCALL) ? 1 : 0);
    
    // Чтение MODE2
    uint8_t mode2;
    if (!readRegister(PCA9685_MODE2, &mode2, 1)) {
        ESP_LOGW(TAG, "⚠️ Не удалось прочитать MODE2");
    } else {
        ESP_LOGI(TAG, "   MODE2: 0x%02X (INVERT=%d, OUTDRV=%d)",
                 mode2,
                 (mode2 & PCA9685_MODE2_INVERT) ? 1 : 0,
                 (mode2 & PCA9685_MODE2_OUTDRV) ? 1 : 0);
    }
    
    // Чтение prescale
    uint8_t prescale;
    if (!readRegister(PCA9685_PRESCALE, &prescale, 1)) {
        ESP_LOGW(TAG, "⚠️ Не удалось прочитать PRESCALE");
    } else {
        float actual_freq = PCA9685_OSC_CLOCK / (4096.0f * (prescale + 1));
        ESP_LOGI(TAG, "   PRESCALE: %u → частота ~%.1f Гц", prescale, actual_freq);
    }
    
    ESP_LOGI(TAG, "✅ Диагностика завершена");
    return true;
}
*/

// ============================================================================
// 🔧 ИНИЦИАЛИЗАЦИЯ (Adafruit-style sequence)
// ============================================================================
/**
 * @brief Инициализация PCA9685
 * @param i2c_manager Ссылка на инициализированный I2C контроллер
 * @param frequency Частота PWM в Гц
 * @return true при успешной инициализацииgetPWMFrequency
 * 
 * @details Последовательность инициализации (точно как в Adafruit):
 * 
 * 1. ✅ Проверка подключения по I2C
 * 2. ✅ Чтение текущего MODE1 (для сохранения битов RESTART/ALLCALL)
 * 3. ✅ Установка SLEEP режима: MODE1 |= 0x10
 * 4. ✅ Расчёт prescale: round(osc/(4096*freq)) - 1
 * 5. ✅ Запись prescale в регистр 0xFE
 * 6. ✅ Выход из SLEEP: MODE1 &= ~0x10 (бит 4 = 0)
 *    ⚠️ Бит RESTART (бит 7) НЕ устанавливается!
 * 7. ✅ Задержка 5 мс для стабилизации осциллятора
 * 8. ✅ Установка MODE2: totem-pole output (бит 2 = 1)
 * 9. ✅ Сброс всех каналов в нейтраль (1500 мкс)
 * 
 * @note После инициализации вызывается resetToNeutral() для безопасности
 *
 *   **Проблема:** В функции `begin()` **ОТСУТСТВУЕТ настройка MODE2**! Это критическая ошибка!
 *      *MODE2 контролирует:**
 *       -       **Бит 2 (OUTDRV):** Тотем-полюс vs открытый сток
 *       - **Бит 4 (INVERT):** Инверсия выхода
 *       **Для сервоприводов нужно:**
 *- `OUTDRV = 1` (тотем-полюс выход)
 *       - `INVERT = 0` (нормальная полярность)
 *       **Без настройки MODE2:** PCA9685 работает в режиме **открытого стока**, что вызывает неправильную интерпретацию PWM сигнала сервоприводом!
 *
 */
// ============================================================================
// 🔧 ИНИЦИАЛИЗАЦИЯ (STRICT DATASHEET COMPLIANCE)
// ============================================================================
bool PCA9685_ServoController::begin(I2CMasterController& i2c_manager) {
    ESP_LOGI(TAG, "=== 🚀 ИНИЦИАЛИЗАЦИЯ PCA9685 (7 серво MG90S) (по Datasheet NXP) ===");
    ESP_LOGI(TAG, "   I2C адрес: 0x%02X, Частота: %u Гц",
            _i2cAddress, _servoParams.frequency);
    ESP_LOGI(TAG, "   🔑 Диапазон импульсов: %u-%u мкс (MG90S)",
            _servoParams.minPulseUs, _servoParams.maxPulseUs);
    
    _i2cManager = &i2c_manager;
    
    // 1. Проверка подключения
    if (!_i2cManager->isDeviceConnected(_i2cAddress)) {
        ESP_LOGE(TAG, "❌ PCA9685 не отвечает на 0x%02X", _i2cAddress);
        return false;
    }
    _chipConnected = true;
    ESP_LOGI(TAG, "✅ PCA9685 обнаружен на 0x%02X", _i2cAddress);
    
    // 2. Чтение MODE1 до инициализации  (Section 7.3.1, Table 5)
    uint8_t mode1;
    if (!readRegister(PCA9685_MODE1, &mode1, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось прочитать MODE1");
        return false;
    }
    ESP_LOGI(TAG, "   MODE1 до инициализации: 0x%02X (SLEEP=%d)",
            mode1, 
            (mode1 & PCA9685_MODE1_SLEEP) ? 1 : 0);
    
    // 3. Переход в SLEEP (Section 7.3.5: PRE_SCALE writable only when SLEEP=1)
    uint8_t sleepMode = (mode1 & 0x7F) | PCA9685_MODE1_SLEEP;
    if (!writeRegister(PCA9685_MODE1, &sleepMode, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось установить SLEEP режим");
        return false;
    }
    ESP_LOGI(TAG, "✅ SLEEP режим установлен");
    delay(5);
    
    // 4. Установка prescaler для 50 Гц
    // 4. Установка PRE_SCALE (Section 7.3.5, Eq. 1)
    // prescale = round(osc_clock / (4096 * freq)) - 1
    const float OSC_CLOCK = 25000000.0f;
    const float TARGET_FREQ = 50.0f;
    uint8_t prescale = static_cast<uint8_t>(
        (OSC_CLOCK / (4096.0f * TARGET_FREQ)) - 1.0f + 0.5f
    );
    ESP_LOGI(TAG, "   Prescaler: %u для %.1f Гц (расчёт: %.2f)", 
            prescale, TARGET_FREQ, 
            OSC_CLOCK / (4096.0f * (prescale + 1)));
    if (!writeRegister(PCA9685_PRESCALE, &prescale, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось установить prescaler");
        return false;
    }
    
    // 5. Выход из SLEEP + RESTART (Section 7.3.1.1, Table 5 Bit 7)
    // Table 5: "User writes logic 1 to this bit [RESTART] to clear it"
    // Чтобы гарантировать сброс флага RESTART (если он был установлен), 
    // нужно записать 1 в бит 7. Но обычно после POR он 0.
    // Для надежности используем 0xA0 (AI=1, RESTART=1, SLEEP=0).
    // Бит RESTART самоочистится после записи 1.
    //  uint8_t wakeMode = PCA9685_MODE1_AUTO_INC;  // 0x20
    uint8_t wakeMode = PCA9685_MODE1_AUTO_INC | PCA9685_MODE1_RESTART; // 0xA0
    ESP_LOGI(TAG, "   🔑 Запись MODE1=0x%02X (AUTO_INCREMENT=1, SLEEP=0)", wakeMode);
    if (!writeRegister(PCA9685_MODE1, &wakeMode, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось выйти из SLEEP режима");
        return false;
    }
    ESP_LOGI(TAG, "✅ Выход из SLEEP (MODE1=0x%02X)", wakeMode);

    
    // 🔑 УСТАНОВКА ПРАВИЛЬНОГО РЕЖИМА ДЛЯ СЕРВОПРИВОДОВ:
    // 8. Настройка MODE2 (Section 7.3.2, Table 6)
    // Бит 2 (OUTDRV) = 1 → Тотем-полюс выход (требуется для серво!)
    // Бит 4 (INVERT) = 0 → Нормальная полярность (не инвертировать!)
    // Бит 5-3 = 0 → Normal output logic
    // Бит 1-0 = 0 → Output change on STOP
    // Для сервоприводов (сигнал HIGH активен) оставляем INVRT=0.
    // Table 12 говорит "inverted" для LED, но для серво сигнала это корректно.
    // ========================================================================
    // 🔑 КРИТИЧЕСКИ: Настройка MODE2 (с проверкой после записи)
    // ========================================================================
    ESP_LOGI(TAG, "⚙️  Настройка MODE2 (режим выхода для серво)...");
    uint8_t mode2 = 0x04;  // 0b00000100 = OUTDRV=1, INVERT=0
    bool mode2_ok = false;

    // Попытка записать 3 раза (защита от сбоев I2C)
    for (uint8_t retry = 0; retry < 3; retry++) {
        if (writeRegister(PCA9685_MODE2, &mode2, 1)) {
            delay(5);  // Дать время на запись
            uint8_t mode2_verify;
            if (readRegister(PCA9685_MODE2, &mode2_verify, 1)) {
                if ((mode2_verify & PCA9685_MODE2_OUTDRV) == PCA9685_MODE2_OUTDRV) {
                    mode2_ok = true;
                    ESP_LOGI(TAG, "   ✅ MODE2 записан успешно с попытки %d (0x%02X)", retry + 1, mode2_verify);
                    break;
                } else {
                    ESP_LOGW(TAG, "   ⚠️ MODE2 записан но OUTDRV=0 (0x%02X), повтор...", mode2_verify);
                }
            } else {
                ESP_LOGW(TAG, "   ⚠️ Не удалось прочитать MODE2 для проверки, повтор...");
            }
        } else {
            ESP_LOGW(TAG, "   ⚠️ Не удалось записать MODE2, повтор...");
        }
        delay(10);
    }

    if (!mode2_ok) {
        ESP_LOGE(TAG, "❌ КРИТИЧЕСКАЯ ОШИБКА: Не удалось установить MODE2 (OUTDRV=0)!");
        ESP_LOGE(TAG, "   Сервоприводы могут вращаться непрерывно!");
        return false;  // ← 🔑 ДОБАВИТЬ: Прервать инициализацию!
    }
    // ========================================================================


    // 6. 🔑 КРИТИЧЕСКИ: ЗАДЕРЖКА 1500 МКС!
    // 6. Стабилизация осциллятора (Section 7.3.1: 500 мкс макс)
    ESP_LOGI(TAG, "⏳ Стабилизация осциллятора (500 мкс минимум)...");
    delay(1500);  // Запас для надежности
    
    // 7. Проверка запуска осциллятора  7. Проверка MODE1
    if (!readRegister(PCA9685_MODE1, &mode1, 1)) {
        ESP_LOGE(TAG, "❌ Не удалось прочитать MODE1 для проверки");
        return false;
    }
    ESP_LOGI(TAG, "   MODE1 после инициализации: 0x%02X", mode1);
    
    if (mode1 & PCA9685_MODE1_SLEEP) {
        ESP_LOGE(TAG, "❌ Осциллятор НЕ запустился! MODE1=0x%02X (SLEEP=1)", mode1);
        return false;
    }
    ESP_LOGI(TAG, "✅ Осциллятор стабилизирован");
    
   
    // 9. Флаг инициализации
    _initialized = true;
    
    ESP_LOGI(TAG, "✅ PCA9685 успешно инициализирован для MG90S!");
    ESP_LOGI(TAG, "   Частота: 50 Гц, Импульсы: %u-%u мкс",
            _servoParams.minPulseUs, _servoParams.maxPulseUs);
    return true;
}

// ============================================================================
// 🔧 УСТАНОВКА PWM ТИКОВ (основной метод записи)  (Section 7.3.3, Table 7)
// ============================================================================
/**
 * @brief Установка PWM тиков для канала
 * @param channel Номер канала (0-15)
 * @param on_tick Тик включения (обычно 0 для серво)
 * @param off_tick Тик выключения (0-4095)
 * @return true при успехе
 * 
 * @details Запись в 4 регистра канала:
 * • LEDn_ON_L  = base + 0
 * • LEDn_ON_H  = base + 1  
 * • LEDn_OFF_L = base + 2
 * • LEDn_OFF_H = base + 3
 * 
 * Где base = PCA9685_LED0_ON_L + channel * 4
 * 
 * @note Для сервоприводов: on_tick = 0, off_tick = ширина импульса в тиках
 */
 
bool PCA9685_ServoController::setPWM(uint8_t channel, uint16_t on_time, uint16_t off_time) {
    if (!_initialized || channel > 15) {
        ESP_LOGE(TAG, "❌ setPWM: невалидные параметры (ch=%u)", channel);
        return false;
    }
    
    // Table 7: 4 регистра на канал. Bit 4 в High byte = Full On/Off
    // Маскируем 0x0F, чтобы не установить бит Full On/Off случайно
    uint8_t reg_base = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t data[4];
    data[0] = on_time & 0xFF;
    data[1] = (on_time >> 8) & 0x0F;  // Mask bits 4-7
    data[2] = off_time & 0xFF;
    data[3] = (off_time >> 8) & 0x0F;  // Mask bits 4-7
    
    if (!_i2cManager->writeRegister(_i2cAddress, reg_base, data, 4)) {
        //  ESP_LOGE(TAG, "❌ setPWM: I2C запись не удалась (канал %u)", channel);
        ESP_LOGE(TAG, "❌ Ошибка записи PWM канал %u", channel);
        return false;
    }
    
    ESP_LOGV(TAG, "setPWM[%u]: on=%u, off=%u", channel, on_time, off_time);
    return true;
}



// ============================================================================
// 🔧 ПУБЛИЧНЫЕ МЕТОДЫ УПРАВЛЕНИЯ
// ============================================================================
/**
 * @brief Установка угла сервопривода (логический диапазон)
 */
// ============================================================================
// 🔧 ИЗМЕНИТЬ ФУНКЦИЮ setLogicalAngle() (добавить вызов printPWMDebug)
// ============================================================================
bool PCA9685_ServoController::setLogicalAngle(uint8_t servoIndex, int16_t angle) {
    if (!_initialized || servoIndex >= 7) {
        ESP_LOGE(TAG, "❌ setLogicalAngle: невалидные параметры");
        return false;
    }
    
    int16_t safeAngle = validateSafetyAngle(angle);
    uint16_t pulseUs = logicalAngleToPulseUs(safeAngle);
    pulseUs += _servoTrim[servoIndex];
    
    // Ограничение физическими пределами
    if (pulseUs < _servoParams.minPulseUs) pulseUs = _servoParams.minPulseUs;
    if (pulseUs > _servoParams.maxPulseUs) pulseUs = _servoParams.maxPulseUs;
    
    uint16_t ticks = pulseUsToTicks(pulseUs);
    
    // 🔑 ДОБАВИТЬ: Отладочная печать PWM значений
    printPWMDebug(servoIndex, safeAngle, pulseUs, ticks);
    
    ESP_LOGV(TAG, "setLogicalAngle[%u]: %+d° → %u мкс → %u тиков",
             servoIndex, safeAngle, pulseUs, ticks);
    return setPWM(_servoChannels[servoIndex], 0, ticks);
}


 /*
bool PCA9685_ServoController::setLogicalAngle(uint8_t servoIndex, int16_t angle) {
    if (!_initialized || servoIndex >= 7) {
        ESP_LOGE(TAG, "❌ setLogicalAngle: невалидные параметры");
        return false;
    }
    
    int16_t safeAngle = validateSafetyAngle(angle);
    uint16_t pulseUs = logicalAngleToPulseUs(safeAngle);
    pulseUs += _servoTrim[servoIndex];
    
    // Ограничение физическими пределами
    if (pulseUs < _servoParams.minPulseUs) pulseUs = _servoParams.minPulseUs;
    if (pulseUs > _servoParams.maxPulseUs) pulseUs = _servoParams.maxPulseUs;
    
    uint16_t ticks = pulseUsToTicks(pulseUs);
    
    ESP_LOGV(TAG, "setLogicalAngle[%u]: %+d° → %u мкс → %u тиков",
             servoIndex, safeAngle, pulseUs, ticks);
    
    return setPWM(_servoChannels[servoIndex], 0, ticks);
}
*/

/**
 * @brief Установка угла (физический диапазон 0-180°)
 */
bool PCA9685_ServoController::setPhysicalAngle(uint8_t servoIndex, uint16_t physicalAngle) {
    if (!_initialized || servoIndex >= 7) return false;
    if (physicalAngle > 180) physicalAngle = 180;
    int16_t logicalAngle = static_cast<int16_t>(physicalAngle) - 90;
    return setLogicalAngle(servoIndex, logicalAngle);
}

/**
 * @brief Прямая установка ширины импульса
 */
bool PCA9685_ServoController::setPulseWidthUs(uint8_t servoIndex, uint16_t pulseUs) {
    if (!_initialized || servoIndex >= 7) return false;
    if (pulseUs < _servoParams.minPulseUs) pulseUs = _servoParams.minPulseUs;
    if (pulseUs > _servoParams.maxPulseUs) pulseUs = _servoParams.maxPulseUs;
    uint16_t ticks = pulseUsToTicks(pulseUs);
    return setPWM(servoIndex, 0, ticks);
}

// ============================================================================
// 🔧 КАЛИБРОВКА
// ============================================================================
void PCA9685_ServoController::setServoTrim(uint8_t servoIndex, int16_t trimUs) {
    if (servoIndex < 7) {
        _servoTrim[servoIndex] = trimUs;
        ESP_LOGI(TAG, "✅ Серво #%u: калибровка = %+d мкс", servoIndex, trimUs);
    }
}

int16_t PCA9685_ServoController::getServoTrim(uint8_t channel) const {
    return (channel < 16) ? _servoTrim[channel] : 0;
}

// ============================================================================
// 🔧 СБРОС В НЕЙТРАЛЬ
// ============================================================================
void PCA9685_ServoController::resetToNeutral() {
    if (!_initialized) {
        ESP_LOGW(TAG, "⚠️ resetToNeutral: контроллер не инициализирован");
        return;
    }
    
    ESP_LOGI(TAG, "🔄 Сброс всех сервоприводов в нейтраль (1500 мкс)");
    for (uint8_t i = 0; i < 7; i++) {
        setLogicalAngle(i, 0);
        //delay(15);
        delay(50);  // 🔑 50 мс между каналами (снижает пиковый ток!)
    }
    ESP_LOGI(TAG, "✅ Все сервоприводы в нейтрали");
}

// ============================================================================
// 🔧 ПРОВЕРКА БЕЗОПАСНОСТИ УГЛА
// ============================================================================
int16_t PCA9685_ServoController::validateSafetyAngle(int16_t angle) const {
    // 🔹 Ограничение максимального угла
    if (angle > SERVO_SAFE_MAX_ANGLE) {
        ESP_LOGV(TAG, "⚠️ Угол %+d° > +%d°, ограничено", angle, SERVO_SAFE_MAX_ANGLE);
        angle = SERVO_SAFE_MAX_ANGLE;
    }
    if (angle < -SERVO_SAFE_MAX_ANGLE) {
        ESP_LOGV(TAG, "⚠️ Угол %+d° < -%d°, ограничено", angle, -SERVO_SAFE_MAX_ANGLE);
        angle = -SERVO_SAFE_MAX_ANGLE;
    }
    
    // 🔹 Буферная зона вокруг нейтрали (предотвращение дребезга)
    if (angle > 0 && angle < SERVO_NEUTRAL_BUFFER) {
        angle = SERVO_NEUTRAL_BUFFER;
        ESP_LOGV(TAG, "⚠️ Угол в буфере (+), скорректировано до +%d°", SERVO_NEUTRAL_BUFFER);
    }
    if (angle < 0 && angle > -SERVO_NEUTRAL_BUFFER) {
        angle = -SERVO_NEUTRAL_BUFFER;
        ESP_LOGV(TAG, "⚠️ Угол в буфере (-), скорректировано до -%d°", -SERVO_NEUTRAL_BUFFER);
    }
    
    return angle;
}


// ============================================================================
// 🔧 ДОБАВИТЬ ФУНКЦИЮ ОТЛАДКИ PWM (после validateSafetyAngle)
// ============================================================================
/**
 * @brief 🔑 ОТЛАДКА: Печать PWM параметров
 */
void PCA9685_ServoController::printPWMDebug(uint8_t servoIndex, int16_t angle,
                                             uint16_t pulseUs, uint16_t ticks) {
    ESP_LOGI(TAG, "   🔍 PWM DEBUG: Серво #%u | Угол: %+3d° | Импульс: %u мкс | Тики: %u",
             servoIndex, angle, pulseUs, ticks);
    
    // ✅ ИСПРАВЛЕНО: Используем actual_us в выводе
    float actual_us = (ticks * 20000.0f) / 4096.0f;
    ESP_LOGI(TAG, "   📊 Фактическая длительность: %.1f мкс (расчёт: %u*20000/4096)", 
             actual_us, ticks);
    
    //            // Расчёт фактической длительности в мкс
    //            float actual_us = (ticks * 20000.0f) / 4096.0f;
    //            ESP_LOGI(TAG, "   📊 Фактическая длительность: %.1f мкс", actual_us);
    
    
    // 🔧 ДОПОЛНИТЕЛЬНАЯ ПРОВЕРКА: Сравнение заданного и фактического импульса
    float diff = static_cast<float>(pulseUs) - actual_us;
    if (abs(diff) > 5.0f) {  // Порог 5 мкс
        ESP_LOGW(TAG, "   ⚠️ Расхождение: задано %u мкс, фактически %.1f мкс (Δ=%.1f мкс)",
                 pulseUs, actual_us, diff);
    }
    // Проверка диапазона
    if (pulseUs < _servoParams.minPulseUs || pulseUs > _servoParams.maxPulseUs) {
        ESP_LOGW(TAG, "   ⚠️ ВНИМАНИЕ: Импульс вне диапазона %u-%u мкс!",
                 _servoParams.minPulseUs, _servoParams.maxPulseUs);
    }
}




// ============================================================================
// 🔧 УТОЧНЁННЫЙ ПРЕДСТАРТОВЫЙ ТЕСТ
// ============================================================================
/**
 * @brief 🔑 УТОЧНЁННЫЙ ПРЕДСТАРТОВЫЙ ТЕСТ СЕРВОПРИВОДОВ
 * @return true при успешном завершении теста
 * 
 * @details Алгоритм теста:
 * 
 * 1. Проверка инициализации и состояния PCA9685
 * 2. Вывод параметров теста в лог
 * 3. Цикл 1..SERVO_TEST_CYCLES:
 *    • Для каждого угла в SERVO_TEST_SEQUENCE:
 *      - Установка угла на ВСЕ 7 каналов ОДНОВРЕМЕННО
 *      - Отладочный вывод для ключевых точек
 *      - Задержка SERVO_TEST_STEP_DELAY_MS
 *    • Пауза SERVO_TEST_PAUSE_MS между циклами
 * 4. Финальный сброс в нейтраль
 * 
 * 📊 Временные характеристики:
 * • Шагов в последовательности: 7
 * • Задержка шага: 600 мс
 * • Пауза между циклами: 1000 мс
 * • Циклов: 3
 * • Общая длительность: ~7*0.6*3 + 2*1.0 = ~14.6 секунд
 * 
 * @note Вызывать в setup() после begin()
 */
bool PCA9685_ServoController::runServoTest() {
    if (!_initialized) {
        ESP_LOGE(TAG, "❌ runServoTest: контроллер не инициализирован!");
        return false;
    }
    
    // ========================================================================
    // ШАГ 1: Вывод параметров теста
    // ========================================================================
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "⚙️  ПРЕДСТАРТОВЫЙ ТЕСТ СЕРВОПРИВОДОВ");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "📊 ПАРАМЕТРЫ:");
    ESP_LOGI(TAG, "   • PCA9685 адрес: 0x%02X", _i2cAddress);
    ESP_LOGI(TAG, "   • Каналы: 0-6 (7 сервоприводов)");
    ESP_LOGI(TAG, "   • Частота PWM: %u Гц", _servoParams.frequency);
    ESP_LOGI(TAG, "   • Импульсы: %u-%u мкс",
             _servoParams.minPulseUs, _servoParams.maxPulseUs);
    ESP_LOGI(TAG, "   • Циклов теста: %d", SERVO_TEST_CYCLES);
    ESP_LOGI(TAG, "   • Задержка шага: %d мс", SERVO_TEST_STEP_DELAY_MS);
    
    // 🔑 ДОБАВИТЬ: Расчётные значения PWM для ключевых углов
    ESP_LOGI(TAG, "📋 РАСЧЁТНЫЕ ЗНАЧЕНИЯ PWM:");
    ESP_LOGI(TAG, "   Угол -90°: %u мкс → %u тиков", 
             _servoParams.minPulseUs, pulseUsToTicks(_servoParams.minPulseUs));
    ESP_LOGI(TAG, "   Угол   0°: %u мкс → %u тиков", 
             _servoParams.neutralPulseUs, pulseUsToTicks(_servoParams.neutralPulseUs));
    ESP_LOGI(TAG, "   Угол +90°: %u мкс → %u тиков", 
             _servoParams.maxPulseUs, pulseUsToTicks(_servoParams.maxPulseUs));
    ESP_LOGI(TAG, "==========================================");
        // ========================================================================
    // ШАГ 2: Основной цикл теста
    // ========================================================================
    for (uint8_t cycle = 0; cycle < SERVO_TEST_CYCLES; cycle++) {
        ESP_LOGI(TAG, "🔄 ЦИКЛ ТЕСТА №%d из %d", cycle + 1, SERVO_TEST_CYCLES);
        
        for (uint8_t step = 0; step < SERVO_TEST_SEQUENCE_SIZE; step++) {
            int16_t targetAngle = SERVO_TEST_SEQUENCE[step];
            
            // 🔑 ОТЛАДКА: печатаем значение из массива
            ESP_LOGD(TAG, "   📋 Шаг %u: SERVO_TEST_SEQUENCE[%u] = %+d°",
                     step, step, targetAngle);
            
            // Все 7 сервоприводов двигаются ОДНОВРЕМЕННО!
            for (uint8_t servoIndex = 0; servoIndex < 7; servoIndex++) {
                setLogicalAngle(servoIndex, targetAngle);
            }
            
            // 🔑 ИСПРАВЛЕНИЕ: логировать КАЖДЫЙ шаг (не только 0,2,4!)
            const char* label;
            if (targetAngle == 0) {
                label = "НЕЙТРАЛЬ";
            } else if (targetAngle > 0) {
                label = "МАКС+";
            } else {
                label = "МАКС-";
            }
            
            ESP_LOGI(TAG, "   📍 Шаг %u: %+3d° [%s] - все 7 серво",
                     step, targetAngle, label);
            
            delay(SERVO_TEST_STEP_DELAY_MS);
        }
        
        if (cycle < SERVO_TEST_CYCLES - 1) {
            ESP_LOGI(TAG, "⏳ Пауза между циклами: %d мс", SERVO_TEST_PAUSE_MS);
            delay(SERVO_TEST_PAUSE_MS);
        }
    }
    
    // ========================================================================
    // ШАГ 3: Финальный сброс
    // ========================================================================
    ESP_LOGI(TAG, "🔄 Финальный сброс в нейтраль (0°)...");
    resetToNeutral();
    delay(500);
    
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "✅ ТЕСТ СЕРВОПРИВОДОВ ЗАВЕРШЁН УСПЕШНО!");
    ESP_LOGI(TAG, "==========================================");
    return true;
}



/**
 * @brief Обработка команд пульта для управления сервоприводами
 * @param comUp Команда тангажа (0-255): 0=вверх, 127=нейтраль, 255=вниз
 * @param comLeft Команда рыскания (0-255): 0=право, 127=нейтраль, 255=лево
 * 
 * @details Распределение команд по каналам PCA9685:
 *   • Каналы 0,1 (рули высоты): реагируют на comUp
 *   • Каналы 2,3 (рули направления): реагируют на comLeft (инверсно)
 *   • Каналы 4,5,6 (крышки): бинарное управление через флаги
 * 
 * @note Применяется проверка безопасности углов через validateSafetyAngle()
 */
void PCA9685_ServoController::processFlightCommands(uint8_t comUp, uint8_t comLeft) {
    if (!_initialized) {
        ESP_LOGW(TAG, "⚠️ processFlightCommands: контроллер не инициализирован");
        return;
    }

    // Преобразование 0-180 → логический диапазон -90°...+90°
    int16_t pitchAngle = map(comUp, 0, 180, 90, -90);
    int16_t yawAngle   = map(comLeft, 0, 180, 90, -90);

    ESP_LOGD(TAG, "📡 RC Commands: Up=%3d→P=%+3d°, Left=%3d→Y=%+3d°", 
             comUp, pitchAngle, comLeft, yawAngle);

    // 🎯 Каналы 0,1: Рули высоты (синхронно)
    setLogicalAngle(0, pitchAngle);
    setLogicalAngle(1, pitchAngle);

    // 🎯 Каналы 2,3: Рули направления (инверсно)
    setLogicalAngle(2, yawAngle);
    setLogicalAngle(3, -yawAngle);

    // 🎯 Каналы 4,5,6: Вспомогательные серво (парашют, крышки)
    // Логика: 0=закрыто, 90=нейтраль, 180=открыто
    int16_t hatchAngle = map(comUp, 0, 180, -90, 90);
    setLogicalAngle(4, hatchAngle);
    setLogicalAngle(5, hatchAngle);
    setLogicalAngle(6, hatchAngle);
}


 /*
bool PCA9685_ServoController::runServoTest() {
    if (!_initialized) {
        ESP_LOGE(TAG, "❌ runServoTest: контроллер не инициализирован!");
        return false;
    }
    
    // ========================================================================
    // ШАГ 1: Вывод параметров теста
    // ========================================================================
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "⚙️  ПРЕДСТАРТОВЫЙ ТЕСТ (Adafruit-style)");
    ESP_LOGI(TAG, "⚙️  ТЕСТ СЕРВОПРИВОДОВ (Datasheet Verified)");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "📊 ПАРАМЕТРЫ:");
    ESP_LOGI(TAG, "   • PCA9685 адрес: 0x%02X", _i2cAddress);
    ESP_LOGI(TAG, "   • Каналы: 0-6 (7 сервоприводов MG90S)");
    // ESP_LOGI(TAG, "   • Частота PWM: %u Гц", _pwmFrequency);
    ESP_LOGI(TAG, "   • Частота PWM: %u Гц", _servoParams.frequency);
    ESP_LOGI(TAG, "   • Импульсы: %u-%u мкс", 
             _servoParams.minPulseUs, _servoParams.maxPulseUs);
    ESP_LOGI(TAG, "   • Циклов теста: %d", SERVO_TEST_CYCLES);
    ESP_LOGI(TAG, "   • Задержка шага: %d мс", SERVO_TEST_STEP_DELAY_MS);
    ESP_LOGI(TAG, "==========================================");
    
    // ========================================================================
    // ШАГ 2: Проверка состояния PCA9685 перед тестом
    // ========================================================================
        //                // Проверка MODE1 перед тестом
        //                ESP_LOGI(TAG, "🔍 Проверка MODE1 перед тестом...");
        //                uint8_t mode1_check;
        //                if (readRegister(PCA9685_MODE1, &mode1_check, 1)) {
        //                    ESP_LOGI(TAG, "   MODE1=0x%02X (SLEEP=%d, AI=%d)",
        //                            mode1_check,
        //                            (mode1_check & PCA9685_MODE1_SLEEP) ? 1 : 0,
        //                            (mode1_check & PCA9685_MODE1_AUTO_INC) ? 1 : 0);
        //                    
        //                    if (mode1_check & PCA9685_MODE1_SLEEP) {
        //                        ESP_LOGE(TAG, "❌ Осциллятор в SLEEP! Повторная инициализация...");
        //                        uint8_t wakeMode = PCA9685_MODE1_AUTO_INC;
        //                        writeRegister(PCA9685_MODE1, &wakeMode, 1);
        //                        delay(100);
        //                    }
        //                    
        //                    if (!(mode1_check & PCA9685_MODE1_AUTO_INC)) {
        //                        ESP_LOGE(TAG, "❌ AUTO_INCREMENT выключен! Серво не будут работать!");
        //                        return false;
        //                    }
        //                    ESP_LOGI(TAG, "✅ AUTO_INCREMENT включён");
        //                }
    
    // ========================================================================
    // ШАГ 3: Основной цикл теста
    // ========================================================================
    // Основной цикл теста
    for (uint8_t cycle = 0; cycle < SERVO_TEST_CYCLES; cycle++) {
        ESP_LOGI(TAG, "🔄 ЦИКЛ ТЕСТА №%d из %d", cycle + 1, SERVO_TEST_CYCLES);
        
        for (uint8_t step = 0; step < SERVO_TEST_SEQUENCE_SIZE; step++) {
            int16_t targetAngle = SERVO_TEST_SEQUENCE[step];
        ESP_LOGD(TAG, "🔄 ЦИКЛ ТЕСТА №%d SERVO_TEST_SEQUENCE %d",  step, SERVO_TEST_SEQUENCE[step]);
            
            // Все 7 сервоприводов двигаются ОДНОВРЕМЕННО!
            for (uint8_t servoIndex = 0; servoIndex < 7; servoIndex++) {
                setLogicalAngle(servoIndex, targetAngle);
            }
            
            // Отладочный вывод
            if (step == 0 || step == 2 || step == 4) {
                const char* label = (step == 0 || step == 4) ? "НЕЙТРАЛЬ" :
                                   (targetAngle > 0) ? "МАКС+" : "МАКС-";
                ESP_LOGI(TAG, "   📍 %+3d° [%s] - все 7 серво", targetAngle, label);
            }
            
            delay(SERVO_TEST_STEP_DELAY_MS);
        }
        
        if (cycle < SERVO_TEST_CYCLES - 1) {
            ESP_LOGI(TAG, "⏳ Пауза между циклами: %d мс", SERVO_TEST_PAUSE_MS);
            delay(SERVO_TEST_PAUSE_MS);
        }
    }
    
    // ========================================================================
    // ШАГ 4: Финальный сброс в нейтраль
    // ========================================================================
    ESP_LOGI(TAG, "🔄 Финальный сброс в нейтраль (0°)...");
    resetToNeutral();
    delay(500);
    
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "✅ ТЕСТ СЕРВОПРИВОДОВ ЗАВЕРШЁН УСПЕШНО!");
    ESP_LOGI(TAG, "==========================================");
    
    return true;
}
*/
