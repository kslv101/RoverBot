// UartDriver.h
#pragma once
#include <functional>
#include <string>
#include <atomic>
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
    int m_fd; // File descriptor הכÿ Linux גלוסעמ HANDLE
    PacketCallback m_callback;
    std::atomic<bool> m_connected;

    bool configurePort();
};