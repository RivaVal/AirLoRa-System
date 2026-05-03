
### 📁 ФАЙЛ 2: `telemetry_bridge.py` (RPi Zero 2W)
#**📍 Позиция:** Запускается как системный сервис или в фоне
#**🧩 Алгоритм:** Отдельный поток → неблокирующее чтение UART → парсинг → атомарная запись в tmpfs
#**💻 Код:**

# python
#!/usr/bin/env python3
"""
📡 telemetry_bridge.py — Неблокирующий мост UART → /tmp/telemetry.txt
🔹 Архитектура: Отдельный поток (thread), изолированный от ground_station.py
🔹 Формат: Бинарный пакет 44Б + CRC8 + Framing (AA 55 ... CC 33)
🔹 Запись: Атомарная замена файла в tmpfs (0 задержек, 0 износ SD)
"""
import serial
import struct
import threading
import os
import tempfile
import time

# ============================================================================
# 📦 КОНФИГУРАЦИЯ И ФОРМАТЫ
# ============================================================================
UART_PORT = "/dev/serial0"  # RPi UART0 (проверить dmesg | grep tty)
BAUD_RATE = 115200
TELEM_FILE = "/tmp/telemetry.txt"

# Структура TelemetryPacket_t (совпадает с ESP32)
TELEM_FMT = '<HI BBH B 4f ff ff B b B B'  # 44 байта
TELEM_SIZE = struct.calcsize(TELEM_FMT)

SYNC_HEADER = b'\xAA\x55'
SYNC_FOOTER = b'\xCC\x33'

# ============================================================================
# 🧮 CRC-8 (Poly 0x07, Init 0x00) — зеркальная реализация ESP32
# ============================================================================
def calc_crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc << 1) ^ 0x07 if crc & 0x80 else crc << 1
    return crc & 0xFF

# ============================================================================
# 📄 ФОРМАТИРОВАНИЕ ДЛЯ FFMPEG DRAWTEXT
# ============================================================================
def format_for_overlay(pkt: tuple) -> str:
    """Преобразует бинарный пакет в многострочную строку для наложения на видео"""
    pid, ts, up, left, thr, flags = pkt[0], pkt[1], pkt[2], pkt[3], pkt[4], pkt[5]
    roll, pitch, yaw, alt, spd, lat, lon = pkt[6], pkt[7], pkt[8], pkt[9], pkt[10], pkt[11], pkt[12]
    vbat, pct, rssi, mode = pkt[13], pkt[14], pkt[15], pkt[16]
    
    # Формат: строго внутри кадра, читаемый шрифт
    return (
        f"MODE {mode} | PID {pid:05d}\n"
        f"BAT {vbat:.2f}V ({pct}%)  |  RSSI {rssi}dBm\n"
        f"ATT R:{roll:+.1f} P:{pitch:+.1f} Y:{yaw:+.1f}\n"
        f"ALT {alt:.1f}m  SPD {spd:.1f}m/s\n"
        f"GPS {lat:.5f}  {lon:.5f}\n"
        f"THR {thr}  UP:{up} LEFT:{left}"
    )

# ============================================================================
# 🧵 ПОТОК ЧТЕНИЯ UART (ИЗОЛИРОВАН ОТ ВИДЕО)
# ============================================================================
def uart_reader_thread():
    buf = bytearray()
    print(f"📡 UART Bridge: запуск {UART_PORT}@{BAUD_RATE}")
    
    with serial.Serial(UART_PORT, BAUD_RATE, timeout=0.1) as ser:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if not data:
                time.sleep(0.01)
                continue
            buf.extend(data)

            # 1. Поиск заголовка
            start = buf.find(SYNC_HEADER)
            if start == -1:
                buf.clear()
                continue
            buf = buf[start:]

            # 2. Ожидание полного пакета (header + payload + crc + footer)
            if len(buf) < TELEM_SIZE + 4:
                continue

            payload = buf[2:2+TELEM_SIZE]
            crc_recv = buf[2+TELEM_SIZE]
            footer = buf[2+TELEM_SIZE+1:4+TELEM_SIZE+1]

            # 3. Валидация
            if footer == SYNC_FOOTER and calc_crc8(payload) == crc_recv:
                pkt = struct.unpack(TELEM_FMT, payload)
                text = format_for_overlay(pkt)
                
                # 4. АТОМАРНАЯ ЗАПИСЬ в tmpfs (защита от half-read ffmpeg)
                tmp_path = TELEM_FILE + ".tmp"
                with open(tmp_path, "w") as f:
                    f.write(text)
                os.replace(tmp_path, TELEM_FILE)  # Атомарная замена
                
                # Сдвиг буфера
                buf = buf[4+TELEM_SIZE+1:]
            else:
                buf = buf[1:]  # Пропуск битого байта

if __name__ == "__main__":
    t = threading.Thread(target=uart_reader_thread, daemon=True)
    t.start()
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 UART Bridge остановлен")

