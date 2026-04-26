

//===============================================
//  4. Исправленный  InputController.cpp
//=================================================

// Проект Sender / Receiver (_SENDER_)
//===============================================
//  4. Исправленный  InputController.cpp
//=================================================
//
//
// 
#include "InputControllers.h"
//  #include <Arduino.h>
#include <Arduino.h>
#include <cstdint>

// ================== JOYSTICK CONTROLLER ==================

//JoystickController::JoystickController(uint8_t xPin, uint8_t yPin, uint8_t buttonPin, 
//                                     uint16_t xMiddle, uint16_t yMiddle, uint8_t deadZone) 
JoystickController::JoystickController(uint8_t xPin, uint8_t yPin , 
                                     uint16_t xMiddle, uint16_t yMiddle, uint8_t deadZone) 
    : _xPin(xPin), _yPin(yPin) ,
    // : _xPin(xPin), _yPin(yPin), _buttonPin(buttonPin),
    //  _xMiddle(xMiddle), _yMiddle(yMiddle), _deadZone(deadZone),
      _xMiddle(xMiddle), _yMiddle(yMiddle) ,
      _debugEnabled(false) {
}

void JoystickController::begin() {
    pinMode(_xPin, INPUT);
    pinMode(_yPin, INPUT);
   //  pinMode(_buttonPin, INPUT_PULLUP);
    
    if (_debugEnabled) {
        Serial.println("🎮 JoystickController initialized");
    //    Serial.printf("  Pins: X=%d, Y=%d, Button=%d\n", _xPin, _yPin, _buttonPin);
        Serial.printf("  Pins: X=%d, Y=%d, \n", _xPin, _yPin );
        Serial.printf("  Centers: X=%d, Y=%d\n", _xMiddle, _yMiddle);
        Serial.printf("  Dead zone: %d\n", _deadZone);
    }
}

uint16_t JoystickController::_filterX(uint16_t rawValue) {
    // Простой фильтр для устранения шума
    static uint16_t filtered = 0;
    filtered = (filtered * 0.7) + (rawValue * 0.3);
    return filtered;
}

uint16_t JoystickController::_filterY(uint16_t rawValue) {
    // Простой фильтр для устранения шума
    static uint16_t filtered = 0;
    filtered = (filtered * 0.7) + (rawValue * 0.3);
    return filtered;
}

void JoystickController::update() {
    // Автоматически вызывается в loop, но можно вызывать вручную
}

//  void JoystickController::getValues(uint8_t& comUp, uint8_t& comLeft, uint8_t& buttonState) {
void JoystickController::getValues(uint8_t& comUp, uint8_t& comLeft) {
    // Чтение и фильтрация сырых значений
    // uint8_t buttonState ;
    uint16_t xRaw = _filterX(analogRead(_xPin));
    uint16_t yRaw = _filterY(analogRead(_yPin));
    
    // Преобразование в диапазон 0-180
    uint16_t xPos = map(xRaw, 4095, 0, 0, 180);
    uint16_t yPos = map(yRaw, 0, 4095, 0, 180);
    
    // Применение мертвой зоны
    const uint16_t middleMax = 90 + _deadZone;
    const uint16_t middleMin = 90 - _deadZone;
    
    // Ось Y (Up/Down)
    if ((yPos < middleMax) && (yPos > middleMin)) {
        comUp = 90; // Центр
    } else {
        comUp = yPos;
    }
    
    // Ось X (Left/Right)
    if ((xPos < middleMax) && (xPos > middleMin)) {
        comLeft = 90; // Центр
    } else {
        comLeft = 180 - xPos; // Инвертируем для естественного управления
    }
    
    // Кнопка
    // buttonState = digitalRead(_buttonPin);
    
    if (_debugEnabled) {
        static uint32_t lastDebug = 0;
        if (millis() - lastDebug > 1000) {
    //        Serial.printf("🎮 Joystick - X: %d->%d, Y: %d->%d, Btn: %d\n", 
    //                     xRaw, comLeft, yRaw, comUp, buttonState);
            Serial.printf("🎮 Joystick - X: %d->%d, Y: %d->%d, Btn:\n", 
                         xRaw, comLeft, yRaw, comUp );
            lastDebug = millis();
        }
    }
    ESP_LOGD("Joystick", "Raw: X=%u→%u, Y=%u→%u | Mapped: X=%u, Y=%u", 
         _xPin, xRaw, _yPin, yRaw, xPos, yPos);
   
}

uint8_t JoystickController::getComUp() const {
    // uint8_t comUp, comLeft, buttonState;
    uint8_t comUp, comLeft;
    // const_cast<JoystickController*>(this)->getValues(comUp, comLeft, buttonState);
    const_cast<JoystickController*>(this)->getValues(comUp, comLeft );
    return comUp;
}

uint8_t JoystickController::getComLeft() const {
    // uint8_t comUp, comLeft, buttonState;
    uint8_t comUp, comLeft;
    // const_cast<JoystickController*>(this)->getValues(comUp, comLeft, buttonState);
    const_cast<JoystickController*>(this)->getValues(comUp, comLeft );
    return comLeft;
}

uint8_t JoystickController::getButtonState() const {
//     return digitalRead(_buttonPin);
    return digitalRead(17);
}

void JoystickController::enableDebug(bool enable) {
    _debugEnabled = enable;
}

void JoystickController::printStatus() {
    // uint8_t comUp, comLeft, buttonState;
    uint8_t comUp, comLeft ;
    // getValues(comUp, comLeft, buttonState);
    getValues(comUp, comLeft );
    
    Serial.println("=== JOYSTICK STATUS ===");
    //Serial.printf("ComUp: %d, ComLeft: %d, Button: %d\n", comUp, comLeft, buttonState);
    Serial.printf("ComUp: %d, ComLeft: %d, Button: \n", comUp, comLeft );
    Serial.printf("Raw X: %d, Raw Y: %d\n", analogRead(_xPin), analogRead(_yPin));
    Serial.println("======================");
}

// ================== SLIDE POT CONTROLLER ==================

SlidePotController::SlidePotController(uint8_t pin) 
    : _pin(pin), _debugEnabled(false), _lastValue(0), _lastReadTime(0) {
}

void SlidePotController::begin() {
    pinMode(_pin, INPUT);
    
    if (_debugEnabled) {
        Serial.println("🎚️ SlidePotController initialized");
        Serial.printf("  Pin: %d\n", _pin);
    }
}

uint16_t SlidePotController::_filterValue(uint16_t rawValue) {
    // Фильтр для устранения дрожания
    static uint16_t filtered = 90;
    
    // Быстрая реакция на большие изменения, медленная на маленькие
    uint16_t diff = abs(rawValue - filtered);
    float filterFactor = (diff > 100) ? 0.3 : 0.1;
    
    filtered = (filtered * (1.0 - filterFactor)) + (rawValue * filterFactor);
    return (uint16_t)filtered;
}

void SlidePotController::update() {
    // Можно использовать для периодического обновления
    if (millis() - _lastReadTime > 25) { // Чтение каждые 25мс
        _lastValue = _filterValue(analogRead(_pin));
        _lastReadTime = millis();
    }
}

uint16_t SlidePotController::getValue() {
    SlidePotController::update(); // Обновляем значение если нужно
    return _lastValue;
}

uint16_t SlidePotController::getMappedValue(uint16_t minOut, uint16_t maxOut) {
    uint16_t rawValue = getValue();
    return map(rawValue, 0, 4095, minOut, maxOut);
}

void SlidePotController::enableDebug(bool enable) {
    _debugEnabled = enable;
}

void SlidePotController::printStatus() {
   uint16_t value = getValue();
   uint16_t mapped = getMappedValue(1000, 2000);
    
    Serial.println("=== SLIDE POT STATUS ===");
    Serial.printf("Raw value: %d\n", value);
    Serial.printf("Mapped (1000-2000): %d\n", mapped);
    Serial.printf("Percentage: %ld%%\n", (long)map(value, 0, 4095, 0, 100)); // ★★★★ ИСПРАВЛЕНО ★★★★
    Serial.println("========================");
}
// Глобальные экземпляры
/*
JoystickController joystick;
SlidePotController slidePot;
*/
//==============================================================