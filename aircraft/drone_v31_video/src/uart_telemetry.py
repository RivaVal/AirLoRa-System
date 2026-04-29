# src/uart_telemetry.py (обновлённая версия)
"""
📡 uart_telemetry.py - Приём телеметрии от ESP32-S3
🔹 Читает UART-порт в отдельном потоке, чтобы не блокировать основной цикл захвата видео.
🔹 Поддерживает два режима:
   1. Реальный: чтение из /dev/ttyAMA0 (формат: KEY1=VAL1;KEY2=VAL2)
   2. Мок: автоматическая генерация тестовых данных при отсутствии UART.
🔹 Все данные хранятся в потокобезопасном словаре (get() возвращает копию).
"""

"""
📡 uart_telemetry.py - Приём бинарного UART-протокола от ESP32-S3
🔹 Парсит пакеты формата: [0xAA][0x55][TYPE][LEN][SEQ_H][SEQ_L][DATA...][CRC16]
🔹 Поддерживает телеметрию: BASIC, GPS, EXTENDED + HEARTBEAT
🔹 Автоматически восстанавливает синхронизацию при потере пакетов
"""

import serial
import struct
import threading
import time
from enum import IntEnum
from typing import Optional, Dict, Callable

class MsgType(IntEnum):
    TELEMETRY_BASIC = 0x01
    TELEMETRY_GPS = 0x02
    TELEMETRY_EXTENDED = 0x03
    CMD_VIDEO_REC = 0x10
    CMD_MODE_CHANGE = 0x11
    HEARTBEAT = 0xFF

# Структуры данных (соответствуют C++ на ESP32)
STRUCTS = {
    MsgType.TELEMETRY_BASIC: ('<ffBB', ['altitude', 'battery_voltage', 'flight_mode', 'reserved']),
    # Добавь остальные по мере необходимости
}

def crc16_ccitt(data: bytes) -> int:
    """CRC-16-CCITT (полином 0x1021, init=0xFFFF)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc

class BinaryTelemetryReader:
    def __init__(self, port="/dev/ttyAMA0", baud=921600):
        self.port, self.baud = port, baud
        self.data: Dict[str, any] = {}
        self.running = False
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.callbacks: Dict[MsgType, Callable] = {}
        self.last_heartbeat = 0
        self.seq_expected = 0

    def register_callback(self, msg_type: MsgType, callback: Callable):
        """Регистрация обработчика для типа сообщения"""
        self.callbacks[msg_type] = callback

    def start(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            print(f"✅ UART подключен: {self.port} @ {self.baud} (binary mode)")
        except serial.SerialException:
            print(f"⚠️ Порт {self.port} недоступен. Включён режим МОК-данных.")
            self.ser = None
        self.running = True
        self.thread = threading.Thread(target=self._parse_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.ser: self.ser.close()

    def _find_sync(self, buffer: bytearray) -> int:
        """Поиск синхробайтов 0xAA 0x55 в буфере"""
        for i in range(len(buffer) - 1):
            if buffer[i] == 0xAA and buffer[i+1] == 0x55:
                return i
        return -1

    def _parse_loop(self):
        buffer = bytearray()
        while self.running:
            # Чтение доступных байт
            if self.ser and self.ser.in_waiting:
                buffer.extend(self.ser.read(self.ser.in_waiting))

            # Поиск и парсинг пакетов
            while len(buffer) >= 6:  # Минимальный размер: header(6) + crc(2)
                sync_idx = self._find_sync(buffer)
                if sync_idx < 0:
                    buffer.clear()  # Нет синхронизации — сброс
                    break

                # Удаляем мусор до синхробайтов
                if sync_idx > 0:
                    del buffer[:sync_idx]

                # Чтение заголовка
                if len(buffer) < 6: break
                header = buffer[0:6]
                payload_len = header[3]
                total_len = 6 + payload_len + 2  # header + payload + crc

                # Ждём полный пакет
                if len(buffer) < total_len: break

                # Извлечение пакета
                packet = buffer[0:total_len]
                del buffer[:total_len]

                # Валидация
                if packet[0] != 0xAA or packet[1] != 0x55: continue
                msg_type = MsgType(packet[2])
                seq_num = struct.unpack('<H', packet[4:6])[0]
                payload = packet[6:6+payload_len]
                crc_recv = struct.unpack('<H', packet[6+payload_len:6+payload_len+2])[0]
                crc_calc = crc16_ccitt(packet[:-2])

                if crc_recv != crc_calc:
                    print(f"⚠️ CRC error (seq={seq_num})")
                    continue

                # Обработка
                self._handle_message(msg_type, payload, seq_num)

            # Мок-данные, если нет UART
            if not self.ser:
                self.data = {
                    "ALT": f"{(time.time() % 120):.1f}",
                    "BAT": "3.85",
                    "MODE": "STAB",
                    "GPS": "55.7558N 37.6173E"
                }
            time.sleep(0.01)  # 100 Гц опрос

    def _handle_message(self, msg_type: MsgType, payload: bytes, seq_num: int):
        """Диспетчеризация по типу сообщения"""
        # Проверка последовательности (опционально)
        if seq_num != self.seq_expected and self.seq_expected > 0:
            print(f"⚠️ Seq gap: expected {self.seq_expected}, got {seq_num}")
        self.seq_expected = seq_num + 1

        # Heartbeat
        if msg_type == MsgType.HEARTBEAT:
            self.last_heartbeat = time.time()
            return

        # Парсинг данных
        if msg_type in STRUCTS:
            fmt, fields = STRUCTS[msg_type]
            try:
                values = struct.unpack(fmt, payload)
                parsed = dict(zip(fields, values))

                # Обновление общего словаря телеметрии
                with self.lock:
                    self.data.update({k.upper(): v for k, v in parsed.items()})

                # Вызов зарегистрированного коллбэка
                if msg_type in self.callbacks:
                    self.callbacks[msg_type](parsed)

            except struct.error as e:
                print(f"⚠️ Parse error for {msg_type.name}: {e}")

    def get(self) -> dict:
        """Потокобезопасное получение текущих данных"""
        with self.lock:
            return self.data.copy()

    def is_alive(self, timeout=2.0) -> bool:
        """Проверка, что связь с ESP32 активна (по heartbeat)"""
        return (time.time() - self.last_heartbeat) < timeout
