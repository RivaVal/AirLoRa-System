


// Проект Sender / Receiver (_SENDER_)
//===============================================
//  4. Исправленный  DataPacketBuilder.h
//=================================================
//
//
#ifndef DATA_PACKET_BUILDER_H
#define DATA_PACKET_BUILDER_H

#include <Arduino.h>
#include <cstdint>

#include "Config.h"
#include "CommonTypes.h"
#include "InputControllers.h"

class DataPacketBuilder {
private:
    uint16_t _packetCounter;
    bool _debugEnabled;
    
    // Ссылки на контроллеры ввода
    JoystickController& _joystick;
    SlidePotController& _slidePot;
    
public:
    DataPacketBuilder(JoystickController& joystick, SlidePotController& slidePot);
    
    void begin();
    DataComSet_t buildDataPacket(bool requestAck = false);
    DataComSet_t buildTestPacket();
    void updatePacketCounter(uint16_t counter) { _packetCounter = counter; }
    uint16_t getPacketCounter() const { return _packetCounter; }
    void enableDebug(bool enable);
    void printPacketInfo(const DataComSet_t& packet);
};

#endif

