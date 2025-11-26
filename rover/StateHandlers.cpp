#include "StateHandlers.h"
#include "Logger.h"
#include "Event.h"
#include "EventQueue.h"
#include "Robot.h"
#include "MissionController.h"

StateHandlers::StateHandlers(Robot& robot, EventQueue& eventQueue, MissionController& missionController) : robot(robot), eventQueue(eventQueue), missionController(missionController)
{
    handlerMap = {
        { State::Init, [this](Robot&, EventQueue&) { return this->handleInit(); } },
        { State::Idle, [this](Robot&, EventQueue&) { return this->handleIdle(); } },
        { State::Planning, [this](Robot&, EventQueue&) { return this->handlePlanning(); } },
        { State::ExecutingPath, [this](Robot&, EventQueue&) { return this->handleExecutingPath(); } },
        { State::Docking, [this](Robot&, EventQueue&) { return this->handleDocking(); } },
        { State::EmergencyStop, [this](Robot&, EventQueue&) { return this->handleEmergencyStop(); } }
    };
}

StateHandlers::StateHandler StateHandlers::getHandlerFor(State state) const
{
    auto it = handlerMap.find(state);
    if (it != handlerMap.end())
    {
        return it->second;
    }

    // Если состояние не найдено — возвращаем заглушку
    return [](Robot&, EventQueue&) { return State::Idle; };
}

State StateHandlers::handleInit() const
{
    // Обрабатываем ОДНО событие за тик (не блокируемся)
    auto event = eventQueue.try_pop();
    if (!event) return State::Init; // нет событий — остаёмся

    switch (event->type)
    {
    case EventType::ArduinoError:
    case EventType::RealSenseError:
    case EventType::InternalError:
        log(LogLevel::Warn, "Init: hardware error detected. Awaiting reset.");
        return State::Init;

    case EventType::StartMission:
        // Если инициализация завершена — переходим в Idle
        log(LogLevel::Info, "Initialization completed successfully. Entering Idle state.");
        return State::Idle;

    default:
        // Игнорируем другие события
        break;
    }
    return State::Init;
}

State StateHandlers::handleIdle() const
{
    auto event = eventQueue.try_pop();
    if (!event) return State::Idle;

    switch (event->type)
    {
    case EventType::TargetSelected:
    {
        if (auto target = robot.getTarget())
        {
            log(LogLevel::Info, "Target selected: " + target->name + ". Awaiting Start command.");
        }
        else
        {
            log(LogLevel::Warn, "TargetSelected received, but no target set.");
        }
        return State::Idle;
    }
    case EventType::StartMission:
    {
        int targetId = robot.getTargetId();
        if (targetId < 0)
        {
            log(LogLevel::Warn, "Start received, but no target selected!");
            return State::Idle;
        }
        // Получаем имя цели для логирования
        auto target = robot.getTarget();
        log(LogLevel::Info, "Starting mission to target: " +
            (target ? target->name : "unknown"));
        return State::Planning;
    }

    case EventType::GlobalStop:
    {
        log(LogLevel::Info, "Global stop received (ignored in Idle).");
        return State::Idle;
    }
    case EventType::ArduinoError:
    case EventType::RealSenseError:
    case EventType::InternalError:
    {
        log(LogLevel::Error, "Hardware error in Idle. Entering EmergencyStop.");
        return State::EmergencyStop;
    }

    default:
        break;
    }
    return State::Idle;
}

State StateHandlers::handlePlanning() const
{
    auto event = eventQueue.try_pop();
    if (!event) return State::Planning; // ждём результат планировщика

    // Мы ожидаем ТОЛЬКО результат планирования
    if (event->type == EventType::PathPlanningResult)
    {
        // Извлекаем payload
        const PlanningResultData* result = event->getData<PlanningResultData>();
        if (!result)
        {
            log(LogLevel::Error, "Planning: received PathPlanningResult without data!");
            return State::Idle;
        }

        if (result->success && !result->path.empty())
        {
            //  устанавливаем путь через безопасный метод
            robot.setPath(std::vector<mathLib::Vec2>(result->path)); // копируем
            return State::ExecutingPath;
        }
        else
        {
            // планировщик не смог найти путь
            log(LogLevel::Warn, "Path planning failed. Returning to Idle.");
            return State::Idle;
        }
    }

    // Обработка экстренных событий (приоритет выше планирования)
    switch (event->type)
    {
    case EventType::GlobalStop:
    {
        log(LogLevel::Info, "Global stop during planning. Aborting.");
        return State::Idle;
    }
    case EventType::ArduinoError:
    case EventType::RealSenseError:
    case EventType::InternalError:
    {
        log(LogLevel::Error, "Hardware error during planning. Entering EmergencyStop.");
        return State::EmergencyStop;
    }
    default:
        break;
    }

    return State::Planning;
}

State StateHandlers::handleExecutingPath() const
{
    missionController.update();

    auto event = eventQueue.try_pop();
    if (!event) return State::ExecutingPath; // продолжаем движение

    switch (event->type)
    {
    case EventType::ObstacleDetected:
    {
        // Извлекаем данные препятствия
        const ObstacleData* obstacle = event->getData<ObstacleData>();
        if (obstacle)
        {
            log(LogLevel::Warn, "Obstacle detected at (" +
                std::to_string(obstacle->position.x) + ", " +
                std::to_string(obstacle->position.y) +
                "), distance: " + std::to_string(obstacle->distance) + "m");
        }
        else
        {
            log(LogLevel::Warn, "Obstacle detected (no data).");
        }
        return State::Idle; // Пока возвращаемся в Idle, можно добавить Replanning
    }

    case EventType::PathBlocked:
        log(LogLevel::Warn, "Path blocked. Stopping motion.");
        missionController.stopMission();
        return State::Idle;

    case EventType::TargetInView:
        if (auto target = robot.getTarget())
        {
            missionController.enableVelocityMode(target->position);
        }

        log(LogLevel::Info, "Target in view. Switching to docking.");
        return State::Docking;

    case EventType::DestinationReached:
        log(LogLevel::Info, "Destination reached. Mission complete.");
        robot.clearPath(); // очищаем путь
        return State::Idle;

    case EventType::GlobalStop:
        log(LogLevel::Info, "Global stop during motion. Halting.");
        return State::Idle;

        // Ошибки оборудования
    case EventType::ArduinoError:
    case EventType::RealSenseError:
    case EventType::InternalError:
        log(LogLevel::Error, "Hardware error during motion. Entering EmergencyStop.");
        return State::EmergencyStop;

    default:
        break;
    }
    return State::ExecutingPath;
}

State StateHandlers::handleDocking() const
{
    auto event = eventQueue.try_pop();
    if (!event) return State::Docking;

    switch (event->type)
    {
    case EventType::DestinationReached:
        log(LogLevel::Info, "Docking complete. Returning to Idle.");
        robot.clearPath();
        return State::Idle;

    case EventType::GlobalStop:
        log(LogLevel::Info, "Global stop during docking.");
        return State::Idle;

        // Ошибки
    case EventType::ArduinoError:
    case EventType::RealSenseError:
    case EventType::InternalError:
        log(LogLevel::Error, "Hardware error during docking. Entering EmergencyStop.");
        return State::EmergencyStop;

    default:
        break;
    }
    return State::Docking;
}

State StateHandlers::handleEmergencyStop() const
{
    auto event = eventQueue.try_pop();
    if (!event) return State::EmergencyStop; // остаёмся в стопе

    switch (event->type)
    {
    case EventType::ResetEmergency:
        log(LogLevel::Info, "Emergency reset requested. Returning to Idle.");
        robot.clearPath(); // сбрасываем все планы
        robot.clearTarget(); // сбрасываем цель
        return State::Idle;

    case EventType::GlobalStop:
        // игнорируем, уже в стопе
        return State::EmergencyStop;

        // В EmergencyStop НЕ обрабатываем другие события (TargetSelected, StartMission и т.д.)
        // Оператор должен явно сбросить Emergency

    default:
        log(LogLevel::Debug, "EmergencyStop: ignoring event " + std::to_string(static_cast<int>(event->type)));
        break;
    }
    return State::EmergencyStop;
}