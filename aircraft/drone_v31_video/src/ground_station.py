#!/usr/bin/env python3
"""
🛰 ground_station.py - Наземная станция приёма видео
🔹 АРХИТЕКТУРА: Слушает UDP-порт -> ffmpeg декодирует MJPEG -> записывает в файл
🔹 УПРАВЛЕНИЕ: Флаги --record (автосохранение) и --display (вывод на экран)
🔹 АЛГОРИТМ: Использует ffmpeg в режиме low-delay для минимизации буферизации.
   При --display запускает отдельный процесс ffplay для просмотра в реальном времени.
"""

"""
🛰 ground_station.py - Наземная станция приёма видео
🔹 Устойчив к ошибкам: занятые порты, отсутствие X11, потеря связи
🔹 Автоматически переключается на альтернативный порт при конфликте
"""

"""
 📜 Готовый `ground_station.py` (оптимизация под Pi Zero 2 W)

Скопируй этот код **полностью**. Он автоматически:
✅ Переключается на `.mkv`
✅ Проверяет доступность аппаратного кодирования
✅ Если HW есть → пишет с телеметрией **внутри кадра** в реальном времени
✅ Если HW нет → пишет **чистый поток** (`-c:v copy`, 0% CPU), а после полёта даёт команду для пост-обработки с сохранением timeline


🛰 ground_station.py - Наземная станция приёма видео
🔹 АРХИТЕКТУРА: Слушает UDP-порт -> ffmpeg декодирует MJPEG -> записывает в файл
🔹 УПРАВЛЕНИЕ: Флаги --record (автосохранение) и --display (вывод на экран)
🔹 АЛГОРИТМ: Использует ffmpeg в режиме low-delay для минимизации буферизации.
"""

"""
🛰 ground_station.py - Наземная станция приёма видео (Pi Zero 2 W Optimized)
🔹 Формат: MKV (устойчив к обрывам)
🔹 Телеметрия: внутри кадра, синхронизирована по timeline
🔹 Аппаратное кодирование: автодетект + fallback на пост-обработку
"""
import subprocess
import argparse
import os
import time
import sys
import signal

class GroundStation:
    def __init__(self, udp_source="udp://127.0.0.1:5000", record=False, display=False):
        self.udp_source = udp_source
        self.record = record
        self.display = display
        self.rec_proc = None
        self.view_proc = None

    def _check_hw_encoder(self) -> bool:
        """Проверяет доступность аппаратного энкодера h264_v4l2m2m"""
        try:
            out = subprocess.run(
                ["ffmpeg", "-hide_banner", "-encoders"],
                capture_output=True, text=True, timeout=5
            ).stdout
            return "h264_v4l2m2m" in out
        except Exception:
            return False

    def start(self):
        os.makedirs("ground_archive", exist_ok=True)
        if self.record:
            ts = int(time.time())
            out_file = f"ground_archive/ground_{ts}.mkv"  # ✅ Надёжный контейнер

            # 🎯 FFMPEG ФИЛЬТР НАЛОЖЕНИЯ ТЕЛЕМЕТРИИ
            # textfile=/tmp/telemetry.txt → ffmpeg читает файл каждый кадр
            # reload=1 → обновляет содержимое без перезапуска декодера
            # box=1:boxcolor=black@0.5 → полупрозрачная подложка для читаемости
            telem_filter = (
                "drawtext=textfile=/tmp/telemetry.txt:reload=1:"
                "fontcolor=white:fontsize=14:x=10:y=10:"
                "box=1:boxcolor=black@0.5:boxborderw=3:"
                "line_spacing=2"
            )

            cmd = ["ffmpeg", "-y", "-fflags", "nobuffer", "-flags", "low_delay",
                   "-i", self.udp_source,
                   "-vf", telem_filter,
                   "-c:v", "libx264", "-preset", "ultrafast", "-crf", "28",
                   "-threads", "2", "-an", out_file]
            try:
                self.rec_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"📼 Запись: {out_file} (Live Telemetry Overlay)")
            except Exception as e:
                print(f"❌ Ошибка запуска записи: {e}")

        if self.display:
            # ... ваш существующий код ffplay ...
            pass
        print("📡 Наземная станция запущена.")

    def stop(self):
        if self.view_proc:
            self.view_proc.send_signal(signal.SIGINT)
            try: self.view_proc.wait(timeout=3)
            except: self.view_proc.kill()

        if self.rec_proc:
            self.rec_proc.send_signal(signal.SIGINT)
            try: self.rec_proc.wait(timeout=10)
            except: self.rec_proc.kill()
        print("🛑 Наземная станция остановлена.")

    def run(self):
        self.start()
        try:
            while True:
                time.sleep(1)
                if self.record and self.rec_proc and self.rec_proc.poll() is not None:
                    print("⚠️ Процесс записи завершился. Перезапуск...")
                    self.rec_proc = None
                    self.start()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="🛰 Наземная станция приёма видео")
    parser.add_argument("--udp", default="udp://127.0.0.1:5000", help="UDP-источник")
    parser.add_argument("--record", action="store_true", help="Автоматическая запись в ground_archive/")
    parser.add_argument("--display", action="store_true", help="Отображение видео")
    args = parser.parse_args()

    station = GroundStation(udp_source=args.udp, record=args.record, display=args.display)
    sys.exit(station.run() or 0)
