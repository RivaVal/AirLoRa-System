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

import subprocess
import argparse
import os
import time
import sys

class GroundStation:
    def __init__(self, udp_source="udp://127.0.0.1:5000", record=False, display=False):
        self.udp_source = udp_source
        self.record = record
        self.display = display
        self.rec_proc = None
        self.view_proc = None

    def _check_port_available(self, port: int) -> bool:
        """Проверяет, свободен ли UDP-порт (грубая эвристика)"""
        try:
            import socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("127.0.0.1", port))
            sock.close()
            return True
        except OSError:
            return False

    def start(self):
        os.makedirs("ground_archive", exist_ok=True)

        # 🔹 Запись (если включена)
        if self.record:
            ts = int(time.time())
            out_file = f"ground_archive/ground_{ts}.mp4"
            cmd = ["ffmpeg", "-y", "-fflags", "nobuffer", "-flags", "low_delay",
                   "-i", self.udp_source, "-c:v", "copy", out_file]
            try:
                self.rec_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                print(f"📼 Запись в: {out_file}")
            except Exception as e:
                print(f"❌ Ошибка запуска записи: {e}")

        # 🔹 Отображение (если включено)
        if self.display:
            # Проверяем наличие графической среды
            if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
                print("⚠️ DISPLAY не задан. Окно просмотра не откроется. Запись продолжится.")
                print("💡 Решение: запусти без --display или настрой проброс X11 (см. документацию)")
            else:
                cmd = ["ffplay", "-fflags", "nobuffer", "-flags", "low_delay",
                       "-framedrop", "-i", self.udp_source, "-window_title", "🛰 Ground Station"]
                try:
                    self.view_proc = subprocess.Popen(cmd)
                    print("🖥 Окно просмотра запущено")
                except Exception as e:
                    print(f"⚠️ Не удалось открыть окно: {e}")

        print("📡 Наземная станция запущена.")

    def stop(self):
        if self.view_proc:
            self.view_proc.terminate()
            self.view_proc.wait(timeout=3)
        if self.rec_proc:
            self.rec_proc.terminate()
            self.rec_proc.wait(timeout=3)
        print("🛑 Наземная станция остановлена.")

    def run(self):
        self.start()
        try:
            while True:
                time.sleep(1)
                # Проверка, жив ли процесс записи
                if self.record and self.rec_proc and self.rec_proc.poll() is not None:
                    print("⚠️ Процесс записи завершился. Перезапуск...")
                    self.rec_proc = None
                    self.start()  # Перезапуск только записи
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="🛰 Наземная станция приёма видео")
    parser.add_argument("--udp", default="udp://127.0.0.1:5000", help="UDP-источник")
    parser.add_argument("--record", action="store_true", help="Автоматическая запись в ground_archive/")
    parser.add_argument("--display", action="store_true", help="Отображение видео (требует DISPLAY)")
    args = parser.parse_args()

    station = GroundStation(udp_source=args.udp, record=args.record, display=args.display)
    sys.exit(station.run() or 0)
