#pragma once
#include <functional>
#include <optional>
#include <string>
#include "CommandTypes.h"

class UartDriver
{
public:
    using PacketCallback = std::function<void(const commands::CommandPacket&)>;

    explicit UartDriver(const std::string& port, int baud_rate);

    void processIncomingData();

    // Отправка пакета
    bool sendPacket(const commands::CommandPacket& packet);

    // Установка callback для принятых пакетов
    void setPacketCallback(PacketCallback cb);

    // Цикл чтения (вызывается из отдельного потока)
    void readLoop();

private:
    // Парсинг строки "$GOTO,5,12.45,8.90*CS" в структуру
    std::optional<commands::CommandPacket> parsePacket(const std::string& line);

    PacketCallback m_callback;
    int m_fd; // файловый дескриптор serial
};