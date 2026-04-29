#!/usr/bin/env python3
"""
🚁 camera_pipeline.py - Бортовой видео-пайплайн для Raspberry Pi Zero 2W
🔹 АРХИТЕКТУРА: Захват -> Оверлей -> Кодирование (ffmpeg) -> UDP-стрим
🔹 УПРАВЛЕНИЕ: CLI-флаги позволяют включать/выключать модули без правки кода.
🔹 АЛГОРИТМЫ:
   - MJPEG-over-UDP: устойчив к потере пакетов, мгновенный старт, низкая задержка.
   - Таймер HQ-снимков: срабатывает независимо от FPS, не блокирует пайплайн.
   - Наложение телеметрии: cv2.putText() на каждый кадр (<2мс на CPU).
"""

import cv2
import time
import subprocess
import argparse
import os
import sys
import signal

def signal_handler(sig, frame):
    print("\n🛑 Получен сигнал остановки. Завершаю пайплайн...")
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

from camera_interface import MockCamera, RealCamera
from uart_telemetry import TelemetryReader

class VideoPipeline:
    def __init__(self, use_mock=True, enable_telemetry=True,
                 auto_hq_interval=0, udp_dest="udp://127.0.0.1:5000"):
        self.use_mock = use_mock
        self.enable_telemetry = enable_telemetry
        self.auto_hq_interval = auto_hq_interval  # 0 = выключено
        self.udp_dest = udp_dest

        self.cam = MockCamera() if use_mock else RealCamera()
        self.telemetry = TelemetryReader() if enable_telemetry else None
        self.ffmpeg_proc = None
        self.running = False
        os.makedirs("archive", exist_ok=True)

    def _start_stream(self):
        """
        Запускает ffmpeg как дочерний процесс.
        Принимает сырые BGR-кадры через stdin, кодирует в MJPEG, шлёт по UDP.
        Использование pipe stdin исключает запись на диск и минимизирует задержку.
        """
        cmd = [
            "ffmpeg", "-y", "-f", "rawvideo", "-vcodec", "rawvideo",
            "-s", "640x480", "-pix_fmt", "bgr24", "-r", "30",
            "-i", "-", "-c:v", "mjpeg", "-q:v", "5", "-f", "mjpeg",
            self.udp_dest
        ]
        self.ffmpeg_proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
        print(f"📡 Стрим запущен: {self.udp_dest}")

    def start(self):
        self.cam.start()
        if self.telemetry: self.telemetry.start()
        self._start_stream()
        self.running = True

    def stop(self):
        self.running = False
        if self.ffmpeg_proc and self.ffmpeg_proc.stdin:
            self.ffmpeg_proc.stdin.close()
            self.ffmpeg_proc.wait()
        self.cam.stop()
        if self.telemetry: self.telemetry.stop()
        print("🛑 Бортовой пайплайн остановлен.")

    def run(self):
        self.start()
        frame_count = 0
        last_hq_time = time.time()
        print(f"🚀 Цикл запущен | Телеметрия: {'ON' if self.enable_telemetry else 'OFF'} | Auto-HQ: {self.auto_hq_interval}s")
        try:
            while self.running:
                # 1. Захват кадра из камеры (или эмулятора)
                frame = self.cam.capture_frame()

                # 2. Наложение телеметрии (если включено)
                if self.enable_telemetry and self.telemetry:
                    telem = self.telemetry.get()
                    if telem:
                        # Формируем строку вида: ALT:12.3m | BAT:3.85V | MODE:STAB
                        text = " | ".join(f"{k}:{v}" for k, v in telem.items())
                        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                # Таймер в правом верхнем углу (всегда)
                cv2.putText(frame, time.strftime("%H:%M:%S"), (480, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                # 3. Отправка в UDP-стрим через ffmpeg
                if self.ffmpeg_proc and self.ffmpeg_proc.stdin:
                    try:
                        self.ffmpeg_proc.stdin.write(frame.tobytes())
                    except BrokenPipeError:
                        print("⚠️ ffmpeg разорвал соединение. Перезапуск...")
                        self._start_stream()

                # 4. Автоматическая генерация HQ-снимка (по таймеру)
                if self.auto_hq_interval > 0 and (time.time() - last_hq_time) >= self.auto_hq_interval:
                    fn = f"archive/HQ_{int(time.time())}.jpg"
                    self.cam.capture_still(fn)
                    print(f"📸 HQ-снимок сохранён: {fn}")
                    last_hq_time = time.time()

                frame_count += 1
                time.sleep(1/30)  # Фиксация ~30 FPS
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="🚁 Бортовой видео-пайплайн Pi Zero 2W")
    parser.add_argument("--mock", action="store_true", help="Использовать Mock-камеру (без железа)")
    parser.add_argument("--telemetry", action="store_true", default=True, help="Включить наложение телеметрии")
    parser.add_argument("--no-telemetry", dest="telemetry", action="store_false", help="Отключить телеметрию")
    parser.add_argument("--auto-hq", type=int, default=10, help="Интервал HQ-снимков в сек (0 = выкл)")
    parser.add_argument("--udp", default="udp://127.0.0.1:5000", help="Адрес UDP-потока")
    args = parser.parse_args()

    pipeline = VideoPipeline(
        use_mock=args.mock,
        enable_telemetry=args.telemetry,
        auto_hq_interval=args.auto_hq,
        udp_dest=args.udp
    )
    sys.exit(pipeline.run() or 0)
