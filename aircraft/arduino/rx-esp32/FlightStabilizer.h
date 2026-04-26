


/**
* @file FlightStabilizer.h
* @brief Заголовочный файл для системы стабилизации полёта
* 
* @details Реализует ПИД-стабилизацию полёта БПЛА с использованием IMU 
* и управления сервоприводами для поддержания заданной ориентации.
* 
* @author Embedded Systems Team
* @version 1.1.0 (ИСПРАВЛЕННАЯ ВЕРСИЯ)
* @date 2026-02-19
*/
#pragma once
#ifndef FLIGHT_STABILIZER_H
#define FLIGHT_STABILIZER_H

#include <Arduino.h>
#include <esp_log.h>
#include "CommonTypes.h"
//  #include "GY87_Handler.h"
#include "PCA9685_ServoController.h"
#include "Config.h"
#include "MPU9250_Handler.h"  // ← КРИТИЧЕСКИ: объявление класса MPU9250Handler

/**
* @struct PIDConfig
* @brief Конфигурация ПИД-регулятора
*/
struct PIDConfig {
    float kp = 0.0f;              ///< Пропорциональный коэффициент
    float ki = 0.0f;              ///< Интегральный коэффициент
    float kd = 0.0f;              ///< Дифференциальный коэффициент
    float maxIntegral = 0.0f;     ///< Ограничение интегральной составляющей
    float maxOutput = 0.0f;       ///< Максимальная коррекция (градусы)
};

/**
* @struct PIDState
* @brief Состояние ПИД-регулятора
* 
* @note ДОБАВЛЕНЫ ПОЛЯ: lastError, output (требуется для calculatePID)
*/
struct PIDState {
    float error = 0.0f;           ///< Текущая ошибка
    float integral = 0.0f;        ///< Накопленная интегральная составляющая
    float derivative = 0.0f;      ///< Дифференциальная составляющая
    float lastError = 0.0f;       ///< ← ДОБАВЛЕНО: Ошибка предыдущего шага
    float output = 0.0f;          ///< ← ДОБАВЛЕНО: Выходное значение
};

/**
* @enum StabilizationMode
* @brief Режимы работы системы стабилизации
*/
enum class StabilizationMode : uint8_t {
    MANUAL = 0,        ///< Полное ручное управление (без стабилизации)
    ROLL_PITCH = 1,    ///< Стабилизация крена и тангажа (автогоризонт)
    FULL = 2,          ///< Полная стабилизация (включая рыскание)
    HOVER = 3          ///< Удержание положения (требует GPS/барометр)
};

/**
* @class FlightStabilizer
* @brief Класс для управления стабилизацией полёта БПЛА
* 
* @details Реализует ПИД-стабилизацию полёта с использованием данных IMU
* и управления сервоприводами для поддержания заданной ориентации.
* 
* @note ВАЖНО: Конструктор теперь без параметров, инициализация через begin()
*/
class FlightStabilizer {
public:
    /**
    * @brief Конструктор системы стабилизации (без параметров)
    * 
    * @note Инициализация выполняется через begin()
    */
    FlightStabilizer();
    
    /**
    * @brief Инициализация системы стабилизации
    * 
    * @param gyroHandler Указатель на обработчик гироскопа
    * @param servoController Указатель на контроллер сервоприводов
    * @return true при успешной инициализации, false при ошибке
    */
    //  bool begin(GY87Handler* gyroHandler, PCA9685_ServoController* servoController);
    bool begin(MPU9250Handler* imuHandler, PCA9685_ServoController* servoController);
    
    /**
    * @brief Включение системы стабилизации
    */
    void enable();
    
    /**
    * @brief Отключение системы стабилизации
    */
    void disable();
    
    /**
    * @brief Установка режима стабилизации
    * @param mode Режим стабилизации
    */
    void setMode(StabilizationMode mode);
    
    /**
    * @brief Обновление состояния стабилизации
    * 
    * @param comUp Команда тангажа (0-255)
    * @param comLeft Команда крена (0-255)
    * @return true если стабилизация активна и данные валидны, false в противном случае
    */
    bool update(float comUp, float comLeft);
    
    /**
    * @brief Сброс состояния ПИД-регуляторов
    */
    void resetPIDStates();
    
    /**
    * @brief Проверка, включена ли стабилизация
    * @return true если стабилизация включена, false в противном случае
    */
    bool isEnabled() const { return _enabled; }
    
    /**
    * @brief Получение текущего режима стабилизации
    * @return Текущий режим стабилизации
    */
    StabilizationMode getMode() const { return _mode; }
    
    /**
    * @brief Проверка стабильности положения
    * @return true если положение стабильно
    */
    bool isStable() const;
    
    /**
    * @brief Вывод статуса системы в лог
    */
    void printStatus() const;
    
    /**
    * @brief Получение конфигурации ПИД крена
    * @return Конфигурация ПИД для крена
    */
    PIDConfig getRollPID() const { return _rollPID; }
    
    /**
    * @brief Получение конфигурации ПИД тангажа
    * @return Конфигурация ПИД для тангажа
    */
    PIDConfig getPitchPID() const { return _pitchPID; }
    
    /**
    * @brief Получение конфигурации ПИД рыскания
    * @return Конфигурация ПИД для рыскания
    */
    PIDConfig getYawPID() const { return _yawPID; }
    
    /**
    * @brief Преобразование режима в строку для отладки
    * @param mode Режим стабилизации
    * @return Строковое представление режима
    */
    const char* modeToString(StabilizationMode mode) const;

private:
    // Указатели на внешние объекты (НЕ владеем памятью)
    // GY87Handler* _gyroHandler;              ///< Указатель на обработчик гироскопа
    MPU9250Handler* imuHandler;              ///< Указатель на обработчик гироскопа
    PCA9685_ServoController* _servoController; ///< Указатель на контроллер сервоприводов
    
    bool _initialized;                      ///< Флаг успешной инициализации
    bool _enabled;                          ///< Флаг включения стабилизации
    StabilizationMode _mode;                ///< Текущий режим стабилизации
    
    float _dt;                              ///< Временной шаг (сек)
    uint32_t _lastUpdateTime;               ///< Время последнего обновления
    
    // Целевые углы (из команд пульта)
    float _targetRoll;                      ///< Целевой крен (градусы)
    float _targetPitch;                     ///< Целевой тангаж (градусы)
    float _targetYaw;                       ///< Целевое рыскание (градусы)
    
    // Конфигурации ПИД-регуляторов для разных осей
    PIDConfig _rollPID;
    PIDConfig _pitchPID;
    PIDConfig _yawPID;
    
    // Состояния ПИД-регуляторов
    PIDState _rollState;
    PIDState _pitchState;
    PIDState _yawState;
    
    // Статистика
    struct {
        uint32_t stabilizations = 0;
        uint32_t corrections = 0;
        uint32_t saturationEvents = 0;
        uint32_t invalidDataEvents = 0;
    } _stats;
    
    /**
    * @brief Расчет ПИД-регулятора
    * @param state Состояние регулятора
    * @param config Конфигурация регулятора
    * @param error Ошибка регулирования
    * @return Выходное значение регулятора
    */
    float calculatePID(PIDState& state, const PIDConfig& config, float error);
    
    /**
    * @brief Применение коррекции крена
    * @param correction Величина коррекции (градусы)
    */
    void applyRollCorrection(float correction);
    
    /**
    * @brief Применение коррекции тангажа
    * @param correction Величина коррекции (градусы)
    */
    void applyPitchCorrection(float correction);
    
    /**
    * @brief Применение коррекции рыскания
    * @param correction Величина коррекции (градусы)
    */
    void applyYawCorrection(float correction);
    
    /**
    * @brief Валидация углов от IMU
    * @param roll Крен (градусы)
    * @param pitch Тангаж (градусы)
    * @param yaw Рыскание (градусы)
    */
    void validateAngles(float& roll, float& pitch, float& yaw);
};

#endif //END  FLIGHT_STABILIZER_H
