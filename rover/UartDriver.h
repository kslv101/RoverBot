// UartDriver.h
#pragma once
#include <functional>
#include <string>
#include <atomic>
#include <vector>
#include <queue>
#include "CommandTypes.h"

class UartDriver {
public:
    using PacketCallback = std::function<void(const uint8_t* data, size_t size)>;

    UartDriver(const std::string& port, int baud_rate);
    ~UartDriver();

    bool connect();
    void disconnect();
    bool sendRaw(const uint8_t* data, size_t size);
    void setPacketCallback(PacketCallback callback);
    void processIncomingData();

private:
    std::string m_port;
    int m_baudRate;
    void* m_handle; // HANDLE для Windows
    PacketCallback m_callback;
    std::atomic<bool> m_connected;
};