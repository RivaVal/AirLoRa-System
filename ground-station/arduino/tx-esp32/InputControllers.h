
//===============================================
//  4. Исправленный  InputController.h
//=================================================

// Проект Sender / Receiver (_SENDER_)
//===============================================
//  4. Исправленный  InputController.h
//=================================================
//
//
//Создаем отдельные модули:
//1. Файл: InputControllers.h
#ifndef INPUT_CONTROLLERS_H
#define INPUT_CONTROLLERS_H

#include <Arduino.h>
#include "Config.h"
//  #include <Arduino.h>
#include <cstdint>

class JoystickController {
private:
    //uint8_t _xPin, _yPin, _buttonPin;
    uint8_t _xPin, _yPin ;
    uint16_t _xMiddle, _yMiddle;
    uint8_t _deadZone;
    bool _debugEnabled;
    
    // Фильтрация значений
    uint16_t _filterX(uint16_t rawValue);
    uint16_t _filterY(uint16_t rawValue);
    
public:
    //JoystickController(uint8_t xPin = 35, uint8_t yPin = 34, uint8_t buttonPin = 16, 
    //                  uint16_t xMiddle = 1445, uint16_t yMiddle = 1870, uint8_t deadZone = 11);
    JoystickController(uint8_t xPin = 35, uint8_t yPin = 34 , 
                      uint16_t xMiddle = 1445, uint16_t yMiddle = 1870, uint8_t deadZone = 11);
    
    void begin();
    void update();
    //  void getValues(uint8_t& comUp, uint8_t& comLeft, uint8_t& buttonState);
    void getValues(uint8_t& comUp, uint8_t& comLeft );
    void enableDebug(bool enable);
    void printStatus();
    
    // Геттеры для отдельных значений
    uint8_t getComUp() const;
    uint8_t getComLeft() const;
    uint8_t getButtonState() const;
};

class SlidePotController {
private:
    uint8_t _pin;
    bool _debugEnabled;
    uint16_t _lastValue;
    uint32_t _lastReadTime;
    
    // Фильтрация и калибровка
    uint16_t _filterValue(uint16_t rawValue);
    
public:
    SlidePotController(uint8_t pin = 25);
    
    void begin();
    void update();
    uint16_t getValue(); // Возвращает 0-4095
    uint16_t getMappedValue(uint16_t minOut = 1000, uint16_t maxOut = 2000); // Для comThrottle
    void enableDebug(bool enable);
    void printStatus();
};

// Глобальные экземпляры (опционально)
//extern JoystickController joystick;
//extern SlidePotController slidePot;

#endif

//==============================================================
