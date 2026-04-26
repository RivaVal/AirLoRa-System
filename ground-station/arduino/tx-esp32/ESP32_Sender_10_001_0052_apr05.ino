                
      



//=============   Sender_Project.ino - ФИНАЛЬНАЯ ВЕРСИЯ  =====================
//
// КОД ПЕРЕДАТЧИКА!
// Sender_Project.ino - ОБНОВЛЕННАЯ ВЕРСИЯ С ТЕСТИРОВАНИЕМ ИНТЕРВАЛОВ
//2. Исправленный главный скетч для эксперимента:
//
//===========================================================================
//  4. Sender_Project.ino - финальный эксперимент с улучшениями
// ========================================================================== 
//
/**
 * @file Sender_Project.ino
 * @brief Эксперимент по тестированию минимальных интервалов LoRa передачи
 * 
 * ЦЕЛЬ ЭКСПЕРИМЕНТА:
 * Определить минимальный стабильный интервал отправки пакетов через LoRa
 * с сохранением 100% доставки и точного соблюдения временных интервалов.
 * 
 * МЕТОДИКА:
 * 1. Последовательное тестирование 6 интервалов: 500, 250, 150, 100, 75, 50 мс
 * 2. Длительность каждого теста: 15 секунд
 * 3. Контрольные точки: каждые 10 отправленных пакетов
 * 4. Измерение реальных интервалов между отправками
 * 5. Расчет теоретической максимальной частоты
 * 
 * РЕЗУЛЬТАТЫ (ожидаемые):
 * - Стабильная работа до 20 Гц (50 мс)
 * - Точность интервалов: ±1 мс
 * - Потеря пакетов: 0%
 * - Время обработки: < 5 мс
 * 
 * АППАРАТНАЯ ЧАСТЬ:
 * - Микроконтроллер: ESP32 DevKit
 * - LoRa модуль: SX1278 (868 МГц)
 * - Антенна: 3dBi штыревая
 * - Питание: 3.3В стабилизированное
 * 
 * @version 3.0
 * @date 2024
 */
/**
 * @file Sender_Project.ino
 * @brief Главный файл передатчика с двумя режимами: Эксперимент и Нормальный
 * 
 * РЕЖИМЫ:
 * 1. EXPERIMENT - Тестирование минимальных интервалов
 * 2. NORMAL     - Реальный режим с InputControllers и структурой DataComSet_t
 * 
 * ВЫБОР РЕЖИМА: раскомментируйте нужную строку
 */

// ============================================================================
// ВЫБОР РЕЖИМА РАБОТЫ
// ============================================================================
#include <SPI.h>
#include <RadioLib.h>
#include <Arduino.h>
#include <cstdint>
#include <esp_log.h>

// Заголовочные файлы
#include "Config.h"
#include "CommonTypes.h"
#include "InputControllers.h"
#include "DataPacketBuilder.h"
#include "LoRaCommunicator.h"
  //  #include "LoRaCommunicator_V2.h"
#include "DisplayManager.h"
#include <GyverOLED.h>

//GyverOLED<SSH1106_128x64> oled;


#define LOG_TAG_TX "Transmitter"


// --- ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ---
DataComSet_t currentDataToSend = {};
uint16_t packetSequence = 0;

// Менеджер дисплея (используем стандартные пины I2C ESP32: SDA=21, SCL=22)
// Если у вас другие пины для дисплея, укажите их здесь: DisplayManager oledDisplay(21, 22);
//        constexpr int DISPLAY_SDA_PIN = 21; 
//        constexpr int DISPLAY_SCL_PIN = 22;
DisplayManager oledDisplay;

// Глобальные переменные
SPIClass vspi(VSPI);  // VSPI instance

// Прототип свободной функции
 void printDataComSet(const DataComSet_t& data, const char* prefix);
 void initDataComSet(DataComSet_t &data);
 void testHardwareConnections() ;
 void testTransmission();
 void updateControllerData();

// JoystickController joystick;
// SlidePotController slidePot;
// Контроллеры ввода
JoystickController joystick(Config::Pins::JOYSTICK_X, Config::Pins::JOYSTICK_Y);
SlidePotController slidePot(Config::Pins::SLIDE_POT);

// Построитель пакетов
DataPacketBuilder* packetBuilder = nullptr;

// Передатчик LoRa
FastLoRaTransmitter* radioSender = nullptr;

// Таймеры
TimerMillis sendTimer(Config::Radio::SEND_INTERVAL_MS);
TimerMillis statsTimer(Config::Timing::STATS_PRINT_INTERVAL_MS);
TimerMillis send_DataComSet_Timer(1200);
TimerMillis getStatus_Timer(1200);




// === ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ===
DataComSet_t currentData = {};
//uint16_t packetSequence = 0;extern 
uint32_t lastPrintTime = 0;
// static uint32_t lastSend = 0; // Время отправки последнего пакета
// Глобально (в начале .ino):
static StateTx lastTxState = StateTx::ERROR;

// === ФУНКЦИИ ===
void initDataComSet(DataComSet_t &data) {//
  data.preamble[0] = PACKET_PREAMBLE_1;
  data.preamble[1] = PACKET_PREAMBLE_2;
  data.packet_id = 0;
  data.comUp = 90;
  data.comLeft = 90;
  data.comThrottle = 1500;
  data.comParashut = 0;
  data.timestamp = millis();
  data.comSetAll = 0;
  data.crc8 = 0;
}

// Принудительный сброс шины I2C перед инициализацией
void resetI2CBus(int sdaPin, int sclPin) {
    pinMode(sclPin, OUTPUT);
    pinMode(sdaPin, OUTPUT);
    
    digitalWrite(sclPin, HIGH);
    digitalWrite(sdaPin, HIGH);
    
    // Генерируем 9 тактов, чтобы вытолкнуть устройство из зависания
    for (int i = 0; i < 9; i++) {
        digitalWrite(sclPin, LOW);
        delayMicroseconds(5);
        digitalWrite(sclPin, HIGH);
        delayMicroseconds(5);
    }
    
    // Посылаем STOP сигнал
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(5);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(sdaPin, HIGH);
    
    delay(100);
}


void setup() {
  Serial.begin(115200);
  delay(1000);
  // yald();
  Serial.println("🚀 \n\n..");
  Serial.println("🚀 \n Starting LoRa Sender Setup...");
  ESP_LOGI("SETUP", "Starting LoRa Sender Setup...");

  // Принудительный сброс шины I2C перед инициализацией
  resetI2CBus(Config::Pins::DISPLAY_SDA_PIN, Config::Pins::DISPLAY_SCL_PIN);
  Wire.begin(Config::Pins::DISPLAY_SDA_PIN, Config::Pins::DISPLAY_SCL_PIN);

  // [SETUP] I2C Scan для определения адреса и типа дисплея
  // [ 1100][I][ESP32_Sender_007_014_mar29.ino:190] setup(): [I2C] Scanning bus...
  ESP_LOGI("SETUP", "Scanning I2C bus...");
  Wire.beginTransmission(0x3C);
  if (Wire.endTransmission() == 0) {
      ESP_LOGI("SETUP", "✅ Device found at 0x3C (likely SSD1306/SH1106)");
  } else {
      Wire.beginTransmission(0x3D);
      if (Wire.endTransmission() == 0) {
          ESP_LOGI("SETUP", "✅ Device found at 0x3D");
          // Если адрес 0x3D, нужно изменить Config.h
      } else {
          ESP_LOGE("SETUP", "❌ No I2C device found at 0x3C or 0x3D!");
      }
  }


    // === ИНИЦИАЛИЗАЦИЯ ДИСПЛЕЯ ===
  Serial.println(">>> Start Display Init..."); // Маркер начала
  ESP_LOGI("SETUP", "Starting Display initialization...");
  
  
  if (!oledDisplay.begin()) {
      ESP_LOGE("SETUP", "❌ CRITICAL: Display initialization FAILED!");
      Serial.println("❌ ERROR: Display not found or init failed.");
      Serial.println("   Check wiring (SDA/SCL) and power.");
      // НЕ продолжаем работу с дисплеем, но не зависаем в бесконечном цикле, 
      // чтобы увидеть остальные логи загрузки LoRa.
      // Если критично - раскомментируй while(1); ниже
      // while(1) delay(1000); 
  } else {
      ESP_LOGI("SETUP", "✅ Display module ready.");
      Serial.println("✅ Display initialized successfully.");
      
      // Принудительная отрисовка старта
      initDataComSet(currentData);
      oledDisplay.forceRedraw(currentData); 
  }
  // =============================

  //  [SETUP] Тестовая проверка дисплея
  //  [ 3825][I][ESP32_Sender_007_014_mar29.ino:200] setup(): [DISPLAY] Running display sanity check...
  ESP_LOGI("SETUP", "Running display sanity check...");
  delay(1000);
  oledDisplay.forceRedraw(currentData);  // Повторная отрисовка
  delay(500);
  // Если после этого текст появился - проблема в таймингах update()
  ESP_LOGI("SETUP", "Sanity check complete");



  // === 1. ЯВНАЯ НАСТРОЙКА ПИНОВ LoRa ===
  pinMode(Config::Pins::LORA_CS,   OUTPUT);
  pinMode(Config::Pins::LORA_RST,  OUTPUT);
  pinMode(Config::Pins::LORA_DIO0, INPUT);   // ← КЛЮЧЕВОЙ ПИН!

  /*
    основная проблема с чтением значений датчиков (джойстик, 
    потенциометр) наиболее вероятно связана с **неправильной 
    настройкой аналогового цифрового преобразователя (АЦП) 
    для специфичных пинов ESP32 (например, 34, 35 и другие 
    входные только).**
  **1. Критическая проблема: Инициализация АЦП**

  Как уже было отмечено в фрагменте кода из файла, пины 
  ESP32, такие как 34 и 35 (часто используемые для аналоговых 
  входов), требуют специальной настройки. Значения 0 и 9, 
  которые вы наблюдаете, характерны для неправильно сконфиг
  урированного АЦП.

*   **Решение:** Добавьте следующие строки в начало функции 
`setup()` вашего основного `.ino` файла (например, `Sender_
Project.ino`) **до** инициализации любых `InputController`.

  */

    // ======== 🔴 КРИТИЧЕСКАЯ ИНИЦИАЛИЗАЦИЯ АЦП =========
    ESP_LOGI("ADC", "Initializing ADC for pins 34-39...");
    // Установить разрешение АЦП на 12 бит (0..4095)
    analogReadResolution(12);
    // Установить максимальное усиление (минимальное ослабление) для расширения диапазона измерения до 3.3В
    // Это важно для пинов 34-39, которые по умолчанию имеют ограниченный диапазон.
    analogSetAttenuation(ADC_11db);
    ESP_LOGI("ADC", "ADC initialization complete.");
    // =============================================
    /*
      **Обоснование:** `analogReadResolution(12)` устанавливает 
    выходной диапазон в 0-4095. `analogSetAttenuation(ADC_11db)` 
    позволяет измерять напряжение близкое к VDD (обычно 3.3В) 
    без значительных искажений. Пины 34-39 могут читать только 
    как `INPUT`, но они также нуждаются в этой настройке ослабления 
    для корректного диапазона.

  **2. Калибровка датчиков (Рекомендуется)**

      Хорошей практикой является калибровка центрального положения 
    джойстика при запуске.

  *   **Решение:** Добавьте простую процедуру калибровки сразу после 
  инициализации АЦП. Примерный код:
    */



    // --- ИНИЦИАЛИЗАЦИЯ SPI ДО ИНИЦИАЛИЗАЦИИ LoRa ---
    // Инициализируем VSPI с нашими пинами
    vspi.begin(
          Config::Pins::VSPI_SCLK,
          Config::Pins::VSPI_MISO,
          Config::Pins::VSPI_MOSI,
           -1);       // SS (не используем, будем передавать отдельно)

    Serial.println("SPI VSPI initialized.");
    Serial.println("RadioLib version: " + String(RADIOLIB_VERSION));
    Serial.println("🚀 Передатчик LoRa — Режим с ACK/без ACK");
    
    // Создание построителя пакетов
    packetBuilder = new DataPacketBuilder(joystick, slidePot);
    packetBuilder->begin();

    // Создание передатчика с указанием SPI
  #if LORA_ENABLE_DIO1
    radioSender = new FastLoRaTransmitter(
        Config::Pins::LORA_CS,
        Config::Pins::LORA_RST,
        Config::Pins::LORA_DIO0, // Передаем DIO0
        Config::Pins::LORA_DIO1, // Передаем DIO1
        &vspi,
        packetBuilder
    );
    ESP_LOGI("SETUP", "📤 Transmitter создан с DIO1 (GPIO%d)", Config::Pins::LORA_DIO1);
  #else
    radioSender = new FastLoRaTransmitter(
        Config::Pins::LORA_CS,
        Config::Pins::LORA_RST,
        Config::Pins::LORA_DIO0,
      //  Config::Pins::LORA_DIO1,   // 🔑 Не передаём при LORA_ENABLE_DIO1=0
        &vspi,
        packetBuilder
    );
    ESP_LOGI("SETUP", "📤 Transmitter создан в режиме DIO0-only");
  #endif
    
    // Инициализация передатчика
    if (!radioSender->begin()) {
        Serial.println("❌ Failed to initialize FastLoRaTransmitter!");
        Serial.println("Possible reasons:");
        Serial.println("  - Wrong power supply (must be exactly 3.3V)");
        Serial.println("  - No common ground connection");
        Serial.println("  - Wrong SPI pin connections");
        Serial.println("  - Damaged SX1278 module");
        Serial.println("  - Incorrect pin definitions in Config.h");
        
        // Проверка подключения (диагностика)
        
        while(1) delay(1000); // Остановка при ошибке
    }
    
    Serial.println("✅ Transmitter initialized successfully!");
    // Настройка отладки
    radioSender->setDebugLevel(DebugLevel::DEBUG_STATS);
    radioSender->setSendInterval(50); // <<< ->

    if ( radioSender->getDebugLevel() >= DebugLevel::DEBUG_ALL) testHardwareConnections();
    // Тестовая передача
    if ( radioSender->getDebugLevel() >= DebugLevel::DEBUG_ALL) testTransmission();
   
    
    // Инициализация контроллеров ввода
    joystick.begin();
    slidePot.begin();
    
    Serial.println("✅ Setup completed. Starting main loop...");

  // Инициализировать DataComSet  
  initDataComSet(currentData);
  pinMode(Config::Pins::LED_BUILTIN, OUTPUT); // Настроить ПИН Мигалки
  digitalWrite(Config::Pins::LED_BUILTIN, LOW);


//1. Добавьте в setup() инициализацию АЦП и калибровку:
  // === КРИТИЧЕСКИ ВАЖНАЯ ИНИЦИАЛИЗАЦИЯ АЦП ESP32 ===
  ESP_LOGI("ADC", "Инициализация АЦП для пинов 34-39");
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  analogSetAttenuation(ADC_11db);  // Расширяем диапазон до 3.3V
  
  // === КАЛИБРОВКА ДАТЧИКОВ ===
  ESP_LOGI("Calibration", "Калибровка джойстика (удерживайте в центре!)");
  delay(2000);
  
  uint32_t xSum = 0, ySum = 0, potSum = 0;
  for (int i = 0; i < 50; i++) {
    xSum += analogRead(Config::Pins::JOYSTICK_X);
    ySum += analogRead(Config::Pins::JOYSTICK_Y);
    potSum += analogRead(Config::Pins::SLIDE_POT);
    delay(10);
  }
  
//  uint16_t xCenter = xSum / 50;
//  uint16_t yCenter = ySum / 50;
//  ESP_LOGI("Calibration", "Центр джойстика: X=%u, Y=%u", xCenter, yCenter);
//  ESP_LOGI("Calibration", "Потенциометр (среднее): %u", potSum / 50);
  
  // Обновляем калибровочные значения в контроллерах
  // (требует добавления метода setCenter() в JoystickController)
  
  // === ТЕСТ LEDC (обязательный по требованию №11) НО ТОЛЬКО ДЛЯ ПРИЕМНИКА ===
  // testLEDCChannels();
  
    
    // Показываем начальные данные на дисплее сразу после старта
    initDataComSet(currentData);
    oledDisplay.forceRedraw(currentData); 
    
    // ... конец setup() ...

} // end  setup()
   

/*
// Получить данные  от контроллеров ввода данных
void updateControllerData() {
  uint8_t comUp, comLeft;
  joystick.getValues(comUp, comLeft);
  currentData.comUp = comUp;
  currentData.comLeft = comLeft;
  currentData.comThrottle = slidePot.getMappedValue(1000, 2000);
  currentData.timestamp = millis();
  currentData.packet_id = packetSequence++;

  // Запрос ACK каждые 17 пакетов
  BitMask flags;
  flags.setBit(0, (packetSequence % 17 == 0));
  currentData.comSetAll = flags.getMask();

 // currentData.crc8 = calculateDataCRC(currentData);
}
*/

    //  void loop() {
//===================================================================
void loop() {
    //static uint32_t lastSend = 0;
    // Добавьте в `loop()` диагностику "зависания" АЦП:
        //        // В начало loop()
        //        static uint32_t lastValidRead = millis();
        //        static uint16_t lastX = 0, lastY = 0;

        //        uint16_t xRaw = analogRead(Config::Pins::JOYSTICK_X);
        //        uint16_t yRaw = analogRead(Config::Pins::JOYSTICK_Y);

        //        if (xRaw == lastX && yRaw == lastY && millis() - lastValidRead > 5000) {
        //          ESP_LOGE("ADC", "⚠️ АЦП завис! Пины: X=%u (pin %d), Y=%u (pin %d)", 
        //                  xRaw, Config::Pins::JOYSTICK_X, yRaw, Config::Pins::JOYSTICK_Y);
        //          // Перезагрузка АЦП
        //          analogReadResolution(12);
        //        }
        //        lastX = xRaw; lastY = yRaw;
        //        lastValidRead = millis();

    // Обновление контроллеров ввода
    joystick.update();
    slidePot.update();

    // Внутри loop():
    if ( radioSender->getDebugLevel() >= DebugLevel::DEBUG_PRINT ) {
      if(getStatus_Timer.isReady() ) { 
        StateTx currentTxState = radioSender->getCurrentState();
        if (currentTxState != lastTxState) {
            Serial.printf("📤 TX FSM: ");
            switch (currentTxState) {
                case StateTx::IDLE:         Serial.println("IDLE"); break; //  PREPARING
                case StateTx::PREPARING:    Serial.println("PREPARING"); break; //  PREPARING
                case StateTx::SENDING:      Serial.println("SENDING"); break;
                case StateTx::WAITING_ACK:  Serial.println("WAITING_ACK"); break;
              //  case StateTx::RECEIVING_ACK:Serial.println("RECEIVING_ACK"); break;
                case StateTx::ERROR:        Serial.println("ERROR"); break;
                default:                    Serial.println("UNKNOWN");
            }
            lastTxState = currentTxState;
        }
      }
    }

    updateControllerData();  // Заполняет currentData

    // Обновление передатчика
    radioSender->update();

    // === ОБНОВЛЕНИЕ ДИСПЛЕЯ ===
    // Передаем актуальную структуру currentData
    // Внутри метода update() уже стоит защита от частого обновления и фиксация значений 90/90
    oledDisplay.update(currentData); 
    // ==========================

    digitalWrite(Config::Pins::LED_BUILTIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(Config::Pins::LED_BUILTIN, LOW);

    
    digitalWrite(Config::Pins::LED_BUILTIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(Config::Pins::LED_BUILTIN, LOW);


    // Печать статистики
    if (send_DataComSet_Timer.isReady()) {
        radioSender->printPacketData();
        send_DataComSet_Timer.reset();
    }

    // Печать статистики
    if (statsTimer.isReady()) {
        statsTimer.reset();
        
        const auto& stats = radioSender->getStatsRef();
        Serial.println("📊 TX Stats: Sent=" + String(stats.packetsSent) + 
                      ", Success=" + String(stats.packetsSentSuccess) + 
                      ", Timeouts=" + String(stats.timeouts) + 
                      ", Errors=" + String(stats.errors));
    }


    
    // Неблокирующая задержка
    delay(1);
}

/*
// 3. Добавьте обязательный тест LEDC:
void testLEDCChannels() {
  ESP_LOGI("LEDC_TEST", "Запуск обязательного теста LEDC каналов...");
  
  // Инициализация LEDC
  ledcSetup(Config::LEDCConfig::SERVO_FREQ, 
            Config::LEDCConfig::SERVO_RESOLUTION, 0);
  
  // Тест сервоканалов
  for (int i = 0; i < 5; i++) {
    ledcAttachPin(Config::Pins::servoPins[i], Config::Pins::servoChannels[i]);
    uint32_t duty = (Config::LEDCConfig::SERVO_MIN_PULSE + 
                    Config::LEDCConfig::SERVO_MAX_PULSE) / 2;
    ledcWrite(Config::Pins::servoChannels[i], duty);
    ESP_LOGI("LEDC_TEST", "Серво %d (pin %d): PWM=%u", 
             i, Config::Pins::servoPins[i], duty);
    delay(100);
  }
  
  // Тест моторов
  ledcSetup(Config::LEDCConfig::MOTOR_FREQ, 
            Config::LEDCConfig::MOTOR_RESOLUTION, 1);
  for (int i = 0; i < 2; i++) {
    ledcAttachPin(Config::Pins::motorPins[i], Config::Pins::motorChannels[i]);
    ledcWrite(Config::Pins::motorChannels[i], 128); // 50% duty
    ESP_LOGI("LEDC_TEST", "Мотор %d (pin %d): 50%% duty", 
             i, Config::Pins::motorPins[i]);
    delay(100);
  }
  
  ESP_LOGI("LEDC_TEST", "✅ Тест LEDC завершен успешно");
  delay(500);
}
*/


// 4. Добавьте подробную отладку в updateControllerData():
void updateControllerData() {
  static uint32_t lastDebug = 0;
  uint8_t comUp, comLeft;
  
  joystick.update();
  slidePot.update();
  joystick.getValues(comUp, comLeft);
  
  // Отладка значений датчиков
  if (millis() - lastDebug > 1000) {
    ESP_LOGI("Sensors", "Джойстик: Up=%u, Left=%u | Потенциометр: %u→%u", 
             comUp, comLeft, 
             slidePot.getValue(), 
             slidePot.getMappedValue(1000, 2000));
    lastDebug = millis();
  }
  
  currentData.comUp = comUp;
  currentData.comLeft = comLeft;
  currentData.comThrottle = slidePot.getMappedValue(1000, 2000);
  // ... остальные поля ...
}


void testHardwareConnections() {
    Serial.println("🔍 Hardware Connection Test:");
    Serial.println("CS pin: " + String(Config::Pins::LORA_CS));
    Serial.println("RST pin: " + String(Config::Pins::LORA_RST));
    Serial.println("DIO0 pin: " + String(Config::Pins::LORA_DIO0));
    
    // Проверка пинов
    pinMode(Config::Pins::LORA_CS, OUTPUT);
    pinMode(Config::Pins::LORA_RST, OUTPUT);
    pinMode(Config::Pins::LORA_DIO0, INPUT);
    
    // Проверка питания через сброс
    digitalWrite(Config::Pins::LORA_RST, LOW);
    delay(10);
    digitalWrite(Config::Pins::LORA_RST, HIGH);
    delay(10);
    
    Serial.println("DIO0 state after reset: " + String(digitalRead(Config::Pins::LORA_DIO0)));
}

void testTransmission() {
    Serial.println("🔍 Testing basic transmission...");
    
    // Отправка тестового пакета
    DataComSet_t testPacket;
    testPacket.preamble[0] = 0xAA;
    testPacket.preamble[1] = 0x55;
    testPacket.packet_id = 0;
    testPacket.comUp = 128;
    testPacket.comLeft = 128;
    testPacket.comThrottle = 1500;
    testPacket.comParashut = 0;
    testPacket.timestamp = millis();
    testPacket.comSetAll = 0;
    testPacket.crc8 = calculateCRC8((uint8_t*)&testPacket, sizeof(testPacket) - 1);
    
    radioSender->setData(testPacket);
    radioSender->sendNow();
    
    Serial.println("✅ Test packet prepared for transmission.");
}


  //===================================================================
/*
   // --- СБОР ДАННЫХ ---
    // Заполнение структуры currentDataToSend (например, с джойстика)
    // joystick.read(&currentData.comUp, &currentData.comLeft);
    // slidePot.read(&currentData.comThrottle);
    // и т.д.
    // Упрощённый пример:
    currentDataToSend.preamble[0] = 0xAA;
    currentDataToSend.preamble[1] = 0x55;
    currentDataToSend.packet_id = packetSequence++;
    currentDataToSend.comUp = 128; // Пример
    currentDataToSend.comLeft = 128; // Пример
    currentDataToSend.comThrottle = 1500; // Пример
    currentDataToSend.comParashut = 22; // Пример
    currentDataToSend.timestamp = millis();
    currentDataToSend.comSetAll = 0x17; // Пример

    // Рассчитать CRC8 (предполагается, что функция calculateCRC8 объявлена в CommonTypes.h)
    // uint8_t* dataStart = reinterpret_cast<uint8_t*>(&currentDataToSend);
    // size_t dataLength = sizeof(DataComSet_t) - 1; // Исключаем CRC8
    // currentDataToSend.crc8 = calculateCRC8(dataStart, dataLength);

    // --- УСТАНОВКА И ОТПРАВКА ДАННЫХ ---
    // Метод setData копирует данные во внутреннюю структуру _txData
    radioSender->setData(currentDataToSend);
    // Метод sendNow запускает FSM, которая готовит и отправляет пакет
    radioSender->sendNow(); // Это вызов FSM, внутри которой происходит подготовка и отправка

    // Serial.print("."); // Можно добавить индикацию вызова FSM, а не фактической отправки

    // --- ОТПРАВКА ПАКЕТОВ С ИНТЕРВАЛОМ ---
    // delay(Config::Timing::INTERVAL_TX_MS); // НЕПРАВИЛЬНО - имя константы не найдено
    // delay(Config::Radio::INTERVAL_TX_MS); // Проверьте Config.h, возможно, имя такое
    printDataComSet(currentDataToSend,"LOOP");
    delay(Config::Radio::SEND_INTERVAL_MS); // Используем правильное имя из реального кода

    //  📤 6. Добавление печати каждые 10/16 пакетов
    // В передатчике:
    static uint16_t txCount = 0;
    txCount++;
    if (txCount % 10 == 0) {
      printDataComSet(currentDataToSend, "SENT");
    }
}
*/ 
 //===================================================================


  /*
  static uint32_t lastSend = 0;
  uint32_t now = millis();

  radioSender->update();

  if (now - lastSend >= 50) {
    updateControllerData();
    // radioSender.setData(currentData);
    radioSender.setData(currentData);
    digitalWrite(Config::Pins::LED_BUILTIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(Config::Pins::LED_BUILTIN, LOW);
    lastSend = now;
  }

  // Печать статистики каждые 15 сек
  if (now - lastPrintTime >= 15000) {
    lastPrintTime = now;
    uint32_t sent, acked, timeouts, errors;
    radioSender.getStats(sent, acked, timeouts, errors);
    Serial.println("\n📊 Статистика передатчика:");
    Serial.printf("  Отправлено: %lu\n", sent);
    Serial.printf("  Подтверждено: %lu (%.1f%%)\n", acked, sent ? (acked * 100.0 / sent) : 0.0);
    Serial.printf("  Таймауты: %lu\n", timeouts);
  }

  // В Sender_Project.ino ОТЛАДКА!!!!
  static uint32_t lastStatPrint = 0;
  if (millis() - lastStatPrint >= 1000 && radioSender.getDebugLevel() >= DebugLevel::DEBUG_PRINT) {
    /__*  
     // auto stats = radioSender.getStatsRef();
      const auto& stats = radioSender.getStatsRef(); // чтобы не копировать     
      const char* states[] = {"IDLE", "PREP", "SEND", "W_ACK", "ERR"};
      Serial.printf("[TX FSM] St: %s | Sent: %lu | ACKs: %lu | TO: %lu | Err: %lu | Busy: %lu\n",
                    states[(int)radioSender.getState()],
                    stats.packetsSent,
                    stats.acksReceived,
                    stats.timeouts,
                    stats.errors,
                    stats.busyCount);
      lastStatPrint = millis();
      *__/
    // auto stats = radioSender.getStatsRef(); // или const auto&
      const auto& stats = radioSender.getStatsRef(); // <- Вот так
      Serial.println("📊 Статистика передатчика:");
      Serial.printf(" Отправлено: %lu", stats.packetsSent);
      Serial.printf(" Подтверждено: %lu (%.1f%%)", stats.acksReceived, 
                    stats.packetsSent ? (stats.acksReceived * 100.0 / stats.packetsSent) : 0.0);
      Serial.printf(" Таймауты: %lu", stats.timeouts);
      Serial.printf(" Ошибки: %lu", stats.errors);
      Serial.printf(" Busy: %lu\n", stats.busyCount); // Если добавлено

  }

  delay(1);
}
*/