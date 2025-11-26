#include "UartDriver.h"
#include <optional>

UartDriver::UartDriver(const std::string& port, int baud_rate)
{}

void UartDriver::processIncomingData()
{}

bool UartDriver::sendPacket(const commands::CommandPacket & packet)
{
    return false;
}

void UartDriver::setPacketCallback(PacketCallback cb)
{}

void UartDriver::readLoop()
{}

std::optional<commands::CommandPacket> UartDriver::parsePacket(const std::string & line)
{
    return std::optional<commands::CommandPacket>();
}
