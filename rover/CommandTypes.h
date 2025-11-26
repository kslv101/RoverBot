#pragma once
#include <cstdint>
#include <array>

#pragma pack(push, 1)

//Общие типы команд для Pi и Arduino
namespace commands 
{
    enum class CmdType : uint8_t
    {
        GOTO = 0,      // Waypoint mode
        VEL = 1,       // Velocity mode
        STOP = 2,      // Emergency stop
        ACK = 3,       // Подтверждение
        REACHED = 4,   // Достигнут waypoint
        ODO_REQ = 5    // Запрос одометрии
    };

    // Пакет команды (максимум 32 байта для UART)
    struct CommandPacket
    {
        CmdType type;
        uint8_t cmdId;           // ID для отслеживания
        uint8_t waypointId;      // Для GOTO/REACHED

        // Данные 
        union // (для экономии памяти)
        {
            struct
            {
                float x, y, theta;     // Позиция (м, рад)
                float linearSpeed;    // м/с
                float angularSpeed;   // рад/с
                float tolerance;       // м
            } goto_data;

            struct
            {
                float linear;          // м/с
                float angular;         // рад/с
                uint16_t duration_ms;  // Длительность
            } vel_data;
        };

        // Контрольная сумма (XOR)
        uint8_t checksum;
    };

    // Размер должен быть 1+1+1+4*6+1 = 28 байт
    static_assert(sizeof(CommandPacket) <= 32, "Packet too large!");

} // namespace commands

// Восстанавливаем прежнее выравнивание
#pragma pack(pop)