// CommandTypes.h
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
        STRAIGHT = 0x01, //движение по прямой
        ROTATE = 0x02    // поворот на месте
    };

    // Базовая структура для всех команд
    struct CommandHeader
    {
        uint8_t startByte;    // 0xAA - маркер начала
        CommandType commandType;
        uint8_t packetId;     // ID пакета для подтверждения
    };

    // Команда движения (объединённая для движения и поворотов)
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

//#pragma once
//#include <cstdint>
//#include <array>
//
//#pragma pack(push, 1)
//
////Общие типы команд для Pi и Arduino
//namespace commands 
//{
//    enum class CmdType : uint8_t
//    {
//        GOTO = 0,      // Waypoint mode
//        VEL = 1,       // Velocity mode
//        STOP = 2,      // Emergency stop
//        ACK = 3,       // Подтверждение
//        REACHED = 4,   // Достигнут waypoint
//        ODO_REQ = 5    // Запрос одометрии
//    };
//
//    // Пакет команды (максимум 32 байта для UART)
//    struct commands::CommandPacket
//    {
//        CmdType type;
//        uint8_t cmdId;           // ID для отслеживания
//        uint8_t waypointId;      // Для GOTO/REACHED
//
//        // Данные 
//        union // (для экономии памяти)
//        {
//            struct
//            {
//                float x, y, theta;     // Позиция (м, рад)
//                float linearSpeed;    // м/с
//                float angularSpeed;   // рад/с
//                float tolerance;       // м
//            } goto_data;
//
//            struct
//            {
//                float linear;          // м/с
//                float angular;         // рад/с
//                uint16_t duration_ms;  // Длительность
//            } vel_data;
//        };
//
//        // Контрольная сумма (XOR)
//        uint8_t checksum;
//    };
//
//    // Размер должен быть 1+1+1+4*6+1 = 28 байт
//    static_assert(sizeof(commands::CommandPacket) <= 32, "Packet too large!");
//
//} // namespace commands
//
//// Восстанавливаем прежнее выравнивание
//#pragma pack(pop)

