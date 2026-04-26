



// Проект Sender / Receiver (_SENDER_)
//===============================================
//  4. Исправленный   DataPacketBuilder.cpp
//=================================================
//
//  5. Файл: DataPacketBuilder.cpp
//  
#include "DataPacketBuilder.h"
#include <Arduino.h>
  // #include <Arduino.h>
#include <cstdint>
#include "Config.h"

DataPacketBuilder::DataPacketBuilder(JoystickController& joystick, SlidePotController& slidePot) 
    : _packetCounter(0), _debugEnabled(false), _joystick(joystick), _slidePot(slidePot) {
}

void DataPacketBuilder::begin() {
    _packetCounter = 0;
    if (_debugEnabled) {
        Serial.println("📦 DataPacketBuilder initialized");
    }
}

//  Файл: DataPacketBuilder.cpp - исправить логику packet_id
//  TODO::
//  В методе buildDataPacket класса DataPacketBuilder счетчик пакетов 
//  увеличивается при каждом вызове. Это правильно, 
//  но нужно убедиться, что он не переполняется.
//  Исправьте функцию buildDataPacket():
DataComSet_t DataPacketBuilder::buildDataPacket(bool requestAck) {
    DataComSet_t packet;
    
    // Защита от переполнения
    // Защита от переполнения (0-65535)
    _packetCounter = (_packetCounter >= 65535) ? 0 : _packetCounter + 1;
    packet.packet_id = _packetCounter;
    
    // Инкремент и присвоение
    //packet.packet_id = ++_packetCounter;

    // Преамбула
    // Преамбула из Config.h
    packet.preamble[0] = PACKET_PREAMBLE_1;  // PACKET_PREAMBLE_1  = 0xAA;
    packet.preamble[1] = PACKET_PREAMBLE_2;  // PACKET_PREAMBLE_2  = 0x55;
     
    //packet.preamble[0] = 0xAA; // PACKET_PREAMBLE_1
    //packet.preamble[1] = 0x55; // PACKET_PREAMBLE_2
    
    // Данные с джойстика
    uint8_t comUp, comLeft ;
    //_joystick.getValues(comUp, comLeft, buttonState);
    _joystick.getValues(comUp, comLeft );
    packet.comUp = comUp;
    packet.comLeft = comLeft;
    
    // Данные с слайд-пота
    packet.comThrottle = _slidePot.getMappedValue(1000, 2000);
    
    // Парашют  //********************************************************
    //packet.comParashut = (buttonState == LOW) ? 1 : 0;
    packet.comParashut = 0 ;
    
    // Временная метка
    packet.timestamp = millis();
    
    // Запрос ACK
    BitMask mask;
    if (requestAck) {
        mask.setBit(0, true);
    }
    packet.comSetAll = mask.getMask();
    
    // CRC будет рассчитан позже
    packet.crc8 = 0;
    
    if (_debugEnabled) {
        printPacketInfo(packet);
    }
    
    return packet;
}


/* УСТАРЕЛ_СТАРЫЙ  
DataComSet_t DataPacketBuilder::buildDataPacket(bool requestAck) {
    DataComSet_t packet;
    
    // Преамбула
    packet.preamble[0] = PACKET_PREAMBLE_1;
    packet.preamble[1] = PACKET_PREAMBLE_2;
    
    // Счетчик пакетов
    packet.packet_id = _packetCounter++;
    
    // Данные с джойстика
    uint8_t comUp, comLeft, buttonState;
    _joystick.getValues(comUp, comLeft, buttonState);
    packet.comUp = comUp;
    packet.comLeft = comLeft;
    
    // Данные с слайд-пота (преобразованные в диапазон 1000-2000)
    packet.comThrottle = _slidePot.getMappedValue(1000, 2000);
    
    // Парашют (кнопка джойстика)
    packet.comParashut = (buttonState == LOW) ? 1 : 0;
    
    // Временная метка
    packet.timestamp = millis();
    
    // Запрос ACK
    BitMask mask;
    if (requestAck) {
        mask.setBit(0, true);
    }
    packet.comSetAll = mask.getMask();
    
    // CRC будет рассчитан позже при отправке
    packet.crc8 = 0;
    
    if (_debugEnabled) {
        printPacketInfo(packet);
    }
    
    return packet;
}
*/

DataComSet_t DataPacketBuilder::buildTestPacket() {
    DataComSet_t packet;
    
    packet.preamble[0] = PACKET_PREAMBLE_1;
    packet.preamble[1] = PACKET_PREAMBLE_2;
    packet.packet_id = _packetCounter++;
    packet.comUp = 90;
    packet.comLeft = 90;
    packet.comThrottle = 1500;
    packet.comParashut = 0;
    packet.timestamp = millis();
    packet.comSetAll = 0b00000001; // Тестовый ACK запрос
    packet.crc8 = 0;
    
    return packet;
}

void DataPacketBuilder::enableDebug(bool enable) {
    _debugEnabled = enable;
}

void DataPacketBuilder::printPacketInfo(const DataComSet_t& packet) {
    Serial.println("=== DATA PACKET INFO ===");
    Serial.printf("Packet ID: %d\n", packet.packet_id);
    Serial.printf("ComUp: %d, ComLeft: %d\n", packet.comUp, packet.comLeft);
    Serial.printf("ComThrottle: %d\n", packet.comThrottle);
    Serial.printf("ComParashut: %d\n", packet.comParashut);
    Serial.printf("Timestamp: %lu\n", packet.timestamp);
    Serial.printf("ComSetAll: 0x%02X\n", packet.comSetAll);
    Serial.println("========================");
}

