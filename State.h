#include <cstdint>
// ��������� ��������� ������
enum class State : std::uint8_t
{
    Init, //������������� ��������, �������� ������� ���������
    Idle, //��������
    Checkup, //��������
    PlanningRoute,//������������ ��������
    Moving,
    LastMileMoving,
    Wait, //��������
    RealSense,
    ReadUpr,
    SendUpr
};