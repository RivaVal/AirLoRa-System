#pragma once
#include "telemetry_protocol.h"
#include <string>
#include <functional>

class UartTelemetry {
public:
    bool open(const std::string& port, int baudrate = 921600);
    void close();
    bool poll(std::function<void(const TelemetryFrame&)> callback);
private:
    int fd_ = -1;
};

