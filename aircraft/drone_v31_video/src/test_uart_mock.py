#!/usr/bin/env python3
"""Тест бинарного протокола без реального UART"""
import struct, time
from uart_telemetry import BinaryTelemetryReader, MsgType, crc16_ccitt

def send_mock_packet(ser, msg_type, payload: bytes):
    """Эмуляция отправки пакета от ESP32"""
    header = struct.pack('<BBB BH', 0xAA, 0x55, msg_type, len(payload), 0)  # seq=0 для теста
    crc = crc16_ccitt(header + payload)
    ser.write(header + payload + struct.pack('<H', crc))

# Виртуальный COM-порт (для теста на хосте)
# Установи: sudo apt install socat
# Запусти в другом терминале: socat -d -d pty,raw,echo=0 pty,raw,echo=0
reader = BinaryTelemetryReader(port="/dev/pts/0", baud=921600)  # Замени на свой pts
reader.start()

# Эмуляция отправки телеметрии
while True:
    payload = struct.pack('<ffBB', 12.5, 3.85, 0, 0)  # TelemetryBasic
    send_mock_packet(reader.ser, MsgType.TELEMETRY_BASIC, payload)
    print("📤 Sent mock telemetry:", reader.get())
    time.sleep(1)
