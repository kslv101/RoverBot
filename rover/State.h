#pragma once
#include <cstdint>
#include <string>

// ��������� ��������� ������
enum class State : std::uint8_t
{
    Init, // �������������
    Idle, // �������� ������� ��������� (����)
    Checkup, // ��������

    PlanningRoute, // ������������ ��������
    MovingToTarget, //���� �� ����, ��������� �����������
    AvoidingObstacle, //������������� � ��������� ��� ���
    ApproachingTarget,// ����������� � ���� (����������� �� ������)

    Wait, //��������
    EmergencyStop, // ���������� ���������

    RealSense,
    ReadControl,
    SendControl
};

std::string toString(State state);
