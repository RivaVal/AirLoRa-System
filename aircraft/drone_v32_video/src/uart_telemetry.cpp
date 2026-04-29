#include "uart_telemetry.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

bool UartTelemetry::open(const std::string& port, int baudrate) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) { perror("UART open"); return false; }

    termios tty{};
    tcgetattr(fd_, &tty);
    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_lflag = 0; tty.c_oflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
    tcsetattr(fd_, TCSANOW, &tty);
    tcflush(fd_, TCIOFLUSH);
    return true;
}

void UartTelemetry::close() { if (fd_ >= 0) ::close(fd_); }

bool UartTelemetry::poll(std::function<void(const TelemetryFrame&)> callback) {
    uint8_t buf[64];
    ssize_t n = read(fd_, buf, sizeof(buf));
    if (n <= 0) return false;

    for (int i = 0; i <= n - (int)sizeof(TelemetryFrame); i++) {
        if (buf[i] == TELEMETRY_SYNC1 && buf[i+1] == TELEMETRY_SYNC2) {
            TelemetryFrame frame;
            std::memcpy(&frame, &buf[i], sizeof(frame));
            if (calc_crc16((uint8_t*)&frame, sizeof(frame) - 2) == frame.crc16) {
                callback(frame);
                return true;
            }
        }
    }
    return false;
}

