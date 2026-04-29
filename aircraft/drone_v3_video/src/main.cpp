#include "uart_telemetry.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::cout << "🚀 Drone Camera Pipeline starting...\n";
    UartTelemetry uart;
    // uart.open("/dev/ttyAMA0"); // Раскомментируй при запуске на RPi

    int frames = 0;
    bool running = true;

    while (running) {
        // Читаем телеметрию от ESP32
        uart.poll([](const TelemetryFrame& t) {
            std::cout << "📊 ALT:" << t.altitude_m << "m | BAT:" << t.battery_v << "V | SEQ:" << t.seq << "\n";
        });

        // Эмуляция захвата кадра (затем замени на picamera2 / libcamera)
        frames++;
        if (frames % 150 == 0) {
            std::cout << "📸 HQ frame saved to archive/\n";
        }

        // Здесь будет код кодирования и отправки в UDP/G4/5G
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }
    return 0;
}
