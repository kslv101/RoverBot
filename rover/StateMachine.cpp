#include "StateMachine.h"

static inline bool consume(std::atomic<bool>& evt)
{
    return evt.exchange(false, std::memory_order_acq_rel); // �������� ��������� � ���������� false
}

State trigger(State state, Robot& rover)
{
    // ���� ���� ����������� ������ � ����� � Init (�� ������� �������������)
    if (rover.triggers.arduinoError.load() ||
        rover.triggers.realSenseError.load() ||
        rover.triggers.correctionError.load())
    {
        return State::Idle;
    }
    // ������ � ����������� ������� � �������
    if (consume(rover.missionComplete))
    {
        return State::Idle;
    }
    if (consume(rover.findObstacle))
    {
        return State::Wait; // ��� ������
    }
    if (consume(rover.findTarget))
    {
        return State::ApproachingTarget; // ������� � ������ ������������
    }
    if (consume(rover.start))
    {
        return State::PlanningRoute;
    }
    
    return state; // ������ �� ��������� � ������� � ������� ���������
}

State init(Robot& rover)
{
    logInfo("Init: starting initialization sequence");
    auto startTime = std::chrono::steady_clock::now();

    // ���� ����������� ������ - �������� ����������������
    if (rover.triggers.arduinoError.load() || rover.triggers.realSenseError.load() || rover.triggers.correctionError.load())
    {
        logWarn("Init: persistent error flags present, stay in Init until cleared by operator");
        return State::Init;
    }

    // �������� ������
    logInfo("Allocating buffers...");
    if (!allocateBuffers(rover, rover.params))
    {
        logError("Init: failed to allocate buffers ");
        rover.triggers.internalError.store(true);
        return State::Init;
    }
    logInfo("Buffers allocated");

    bool isConfigOk = true;
    // TODO: �������� �������
    // ���� fail -> config_ok=false;
    if (!isConfigOk)
    {
        logError("Init: config loading failed");
        return State::Init;
    }

    // 3) ������������� serial/Arduino ����������� ���������
    logInfo("Checking serial/Arduino...");
    std::string arduinoVersion;
    bool isSerialOk = false;
    for (int attempt = 1; attempt <= rover.params.serialRetries; ++attempt)
    {
        if (checkSerialDevice("/dev/ttyUSB0", 115200, arduinoVersion))
        {
            isSerialOk = true;
            logInfo("Serial OK. Arduino version: " + arduinoVersion);
            break;
        }
        else
        {
            logWarn("Serial check attempt " + std::to_string(attempt) + " failed.");
            std::this_thread::sleep_for(rover.params.retryDelay * attempt);
        }
        // timeout check
        if (std::chrono::steady_clock::now() - startTime > rover.params.maxInitDuration)
        {
            logError("Init: exceeded max init duration during serial init");
            break;
        }
    }
    if (!isSerialOk)
    {
        logError("Init: serial init failed -> mark arduinoError");
        rover.triggers.arduinoError.store(true); 
        return State::Init;
    }

    // ��������� ������
    logInfo("Checking camera...");
    bool isCameraOk = false;
    for (int attempt = 1; attempt <= rover.params.cameraRetries; ++attempt)
    {
        if (checkCameraDevice(0))
        {
            isCameraOk = true;
            break;
        }
        else
        {
            logWarn("Camera check attempt " + std::to_string(attempt) + " failed.");
            std::this_thread::sleep_for(rover.params.retryDelay * attempt);
        }
        if (std::chrono::steady_clock::now() - startTime > rover.params.maxInitDuration)
        {
            logError("Init: exceeded max init duration during camera init");
            break;
        }
    }
    if (!isCameraOk)
    {
        logError("Init: camera init failed -> mark realSenseError (or camera error)");
        rover.triggers.realSenseError.store(true);
        return State::Init;
    }

    logInfo("Checking RealSense...");
    if (!checkRealSense())
    {
        logWarn("Optional sensors not responding");
        // �� ��������? �������� ������ ����������, �� ����� �������� �������
    }

    // ������� �������� ����������������� ������� 
    {
        // TODO: ��������� ����������� ������ � ��������� � �������� ��������
        bool isSensorsOk = true;
        if (!isSensorsOk)
        {
            logError("Init: sensors sanity check failed");
            rover.triggers.internalError.store(true);
            return State::Init;
        }
    }


    logInfo("All init tasks passed, transitioning to next State");
    // ����� ������
    consume(rover.start);            // ���� ���-�� �������� �������� �����
    consume(rover.findObstacle);
    consume(rover.findTarget);
    consume(rover.missionComplete);

    return State::Idle;
}

State idle(Robot& rover)
{
    if (rover.newCommandReceived.exchange(false))
    {
        int targetId = rover.pendingTargetId.load();
        // �����: ��������� ���� �� ����� �� ID
        log(LogLevel::Info, "Target ID " + std::to_string(targetId) + " selected");
        return State::PlanningRoute;
    }
    if (rover.start.exchange(false))
    {
        return State::PlanningRoute;
    }
    return State::Idle;

    /*int number;
    std::cout << "Select target type" << std::endl;
    std::cout << "0 - Circle\n"
                 "1 - Square\n"
                 "2 - Triangle\n";
    std::cin >> number;

    while (number != 0 && number != 1 && number != 2)
    {
        std::cout << "Select CORRECT target type" << std::endl;
        std::cout << "0 - Circle\n"
                     "1 - Square\n"
                     "2 - Triangle\n";
        std::cin >> number;
    }
    rover.target.type = static_cast<TargetType>(number);*/


    return State::PlanningRoute;
}

State plan(Robot& rover)
{
    if (rover.planningImpossible)
    {
        return State::Idle;
    }
    else
    {
        return State::MovingToTarget;
    }
};

State move(Robot& rover)
{
    return State::Wait;
}

State avoidObstacle(Robot& rover)
{
    return State::Wait;
}

State approachTarget(Robot& rover)
{
    return State::Wait;
}

State wait(Robot& rover)
{
    if (rover.movingOk)
    {
        return State::RealSense;
    }
    else
    {
        return State::Wait;
    }
};

State realSense(Robot& rover)
{
    if (rover.findObstacle)
    {
        return State::PlanningRoute;
    }
    else if (rover.findTarget)
    {
        return State::PlanningRoute;
    }
    else
    {
        return State::PlanningRoute;
    }
};

State readControl(Robot& rover)
{
    if (rover.missionComplete)
    {
        return State::Init;
    }
    else
    {
        return State::SendControl;
    }
};

State sendControl(Robot& rover)
{
    return State::ReadControl;
}