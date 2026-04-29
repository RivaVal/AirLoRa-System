"""
📷 camera_interface.py - Абстрактный слой работы с камерой
🔹 Реализует паттерн Strategy: позволяет легко переключаться между
   реальной CSI-камерой и программным эмулятором без изменения логики пайплайна.
🔹 MockCamera: генерирует тестовые кадры с синтетическими объектами.
🔹 RealCamera: использует picamera2 (только на Raspberry Pi).
"""

import numpy as np
import cv2
import time
from abc import ABC, abstractmethod

class CameraInterface(ABC):
    """Базовый интерфейс камеры. Все реализации должны наследовать этот класс."""
    @abstractmethod
    def start(self): pass
    @abstractmethod
    def stop(self): pass
    @abstractmethod
    def capture_frame(self) -> np.ndarray: pass
    @abstractmethod
    def capture_still(self, filename: str) -> None: pass

class MockCamera(CameraInterface):
    """Эмуляция камеры для отладки на хост-машине (x86_64)."""
    def __init__(self, width=640, height=480):
        self.width, self.height = width, height
        self.running = False
        # Фоновый буфер для оптимизации (не выделяем память каждый кадр)
        self.bg = np.zeros((height, width, 3), dtype=np.uint8)

    def start(self): self.running = True
    def stop(self): self.running = False

    def capture_frame(self):
        """Возвращает тестовый кадр 640x480 с геометрическими объектами и таймером."""
        frame = self.bg.copy()
        cv2.rectangle(frame, (50, 50), (550, 400), (255, 255, 255), 2)
        cv2.circle(frame, (320, 240), 80, (0, 120, 255), 3)
        cv2.putText(frame, "MOCK LIVE FEED", (180, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(frame, time.strftime("%H:%M:%S"), (240, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        return frame

    def capture_still(self, filename):
        """Сохраняет 'HQ-снимок' в разрешении 1920x1080 (эмуляция)."""
        img = np.ones((1080, 1920, 3), dtype=np.uint8) * 30
        cv2.putText(img, "HQ STILL [MOCK]", (300, 540), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        cv2.imwrite(filename, img)

class RealCamera(CameraInterface):
    """Реальная CSI-камера Raspberry Pi (требует установленного picamera2)."""
    def __init__(self):
        try:
            from picamera2 import Picamera2
            self.picam = Picamera2()
            # Базовая конфигурация: 640x480 для стрима, 4 буфера для плавности
            self.config = self.picam.create_video_configuration(main={"size": (640, 480)})
            self.picam.configure(self.config)
        except ImportError:
            raise RuntimeError("picamera2 не найден. Используй флаг --mock для тестов на хосте.")

    def start(self): self.picam.start()
    def stop(self): self.picam.stop()
    def capture_frame(self): return self.picam.capture_array()
    def capture_still(self, filename): self.picam.capture_file(filename)
