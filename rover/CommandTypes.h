#pragma once
#include <cstdint>

#pragma pack(push, 1)
namespace commands {
    enum class CommandType : uint8_t
    {
        MOVE = 0x02,
        STOP = 0x03,
        EMERGENCY_STOP = 0x04,
        GET_STATUS = 0x05
    };

    // Типы движения
    enum MotionType : uint8_t
    {
        STRAIGHT = 0x01, // движение по прямой
        ROTATE = 0x02    // поворот на месте
    };

    // Базовая структура для всех команд
    struct CommandHeader
    {
        uint8_t startByte;    // 0xAA - маркер начала
        CommandType commandType;
        uint8_t packetId;     // ID пакета для подтверждения
    };

    // Команда движения (движение или поворот)
    struct MoveCommand
    {
        CommandHeader header;
        uint8_t segmentId;     // Уникальный ID сегмента (1-255) (для сопоставления с телеметрией)
        MotionType motionType;
        int16_t targetValue;   // Целевое значение: для движения - мм, для поворота - градусы*10
        uint8_t checksum;
    };

    using CommandPacket = MoveCommand;
}
#pragma pack(pop)