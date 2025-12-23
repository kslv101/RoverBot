#include "NetworkCommand.h"
#include "Logger.h"
#include <string>
#include <cstring>
#include <chrono>
#include <thread>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using ssize_t = int;
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fcntl.h>
#endif

#include "../extern/json.hpp"
using json = nlohmann::json;

static std::thread g_listenerThread;
static std::atomic<bool> g_stopListener{ false };
static constexpr int COMMAND_PORT = 5006;

// RAII-обёртка для сокета (автоматическое закрытие)
class SocketGuard
{
public:
    explicit SocketGuard(int sock) : sockfd(sock) {}
    ~SocketGuard()
    {
        if (sockfd >= 0)
        {
        #ifdef _WIN32
            closesocket(sockfd);
        #else
            close(sockfd);
        #endif
        }
    }
    int get() const { return sockfd; }
    bool valid() const { return sockfd >= 0; }

private:
    int sockfd;
    SocketGuard(const SocketGuard&) = delete;
    SocketGuard& operator=(const SocketGuard&) = delete;
};

void handleJsonCommand(Robot& rover, EventQueue& events, const json& cmd)
{
    std::string command = cmd["cmd"].get<std::string>();

    if (command == "set_initial_pose")
    {
        float x = cmd["x"].get<float>();
        float y = cmd["y"].get<float>();
        rover.initialPosition = { x, y };
        rover.setPose(mathLib::Vec2(x, y), 0);
        log(LogLevel::Info, "Initial pose set to (" + std::to_string(x) + ", " + std::to_string(y) + ")");
    }
    else if (command == "plan_path")
    {
        int targetId = cmd["target_id"].get<int>();
        if (targetId >= 0 && targetId < static_cast<int>(rover.map.getTargets().size()))
        {
            rover.selectedTargetId = targetId;
            rover.selectedTarget = rover.map.getTargets()[targetId];
            log(LogLevel::Info, "Target selected: " + rover.selectedTarget.name);
            events.push(Event(EventType::TargetSelected));
        }
        else
        {
            log(LogLevel::Error, "Invalid target ID: " + std::to_string(targetId));
        }
    }
    else if (command == "start")
    {
        events.push(Event(EventType::StartMission));
    }
    else if (command == "stop")
    {
        events.push(Event(EventType::GlobalStop));
        log(LogLevel::Info, "Stop command received");
    }
    else
    {
        log(LogLevel::Warn, "Unknown command: " + command);
    }
}

void udpListenerLoop(Robot& rover, EventQueue& events)
{
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        log(LogLevel::Error, "WSAStartup failed");
        return;
    }
    // RAII для WSACleanup
    struct WSAInitializer
    {
        ~WSAInitializer() { WSACleanup(); }
    } wsaInit;
#endif

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    SocketGuard sockGuard(sockfd);

    if (!sockGuard.valid())
    {
        log(LogLevel::Error, "Failed to create UDP socket");
        return;
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(COMMAND_PORT);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockGuard.get(), (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
    {
        log(LogLevel::Error, "Failed to bind UDP socket");
        return;
    }

    // Установка неблокирующего режима
#ifndef _WIN32
    int flags = fcntl(sockGuard.get(), F_GETFL, 0);
    fcntl(sockGuard.get(), F_SETFL, flags | O_NONBLOCK);
#else
    u_long mode = 1;
    ioctlsocket(sockGuard.get(), FIONBIO, &mode);
#endif

    log(LogLevel::Info, "UDP command listener started on port " + std::to_string(COMMAND_PORT));

    char buffer[1024];
    while (!g_stopListener.load(std::memory_order_relaxed))
    {
        ssize_t len = recvfrom(sockGuard.get(), buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
        if (len > 0)
        {
            buffer[len] = '\0';
            try
            {
                auto j = json::parse(std::string(buffer, len));
                handleJsonCommand(rover, events, j);
            }
            catch (const std::exception& e)
            {
                log(LogLevel::Error, "JSON parse error: " + std::string(e.what()));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void sendRouteToGUI(const std::vector<RoutePoint>& keypoints, const mathLib::Vec2& start, const mathLib::Vec2& goal, const std::string& routeId)
{
    try 
    {
        // Формируем JSON сообщение
        json routeData;
        routeData["type"] = "route_update";
        routeData["route_id"] = routeId.empty() ? "route_" + std::to_string(std::time(nullptr)) : routeId;
        routeData["timestamp"] = std::time(nullptr);

        // Начальная и конечная точки
        routeData["start"] = { {"x", start.x}, {"y", start.y} };
        routeData["goal"] = { {"x", goal.x}, {"y", goal.y} };

        // Ключевые точки
        json keypointsJson = json::array();
        for (const auto& point : keypoints) 
        {
            keypointsJson.push_back({ {"x", point.x}, {"y", point.y} });
        }
        routeData["keypoints"] = keypointsJson;

        // Конвертируем в строку
        std::string jsonData = routeData.dump();

        // Создаём сокет для отправки
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) 
        {
            log(LogLevel::Error, "GUI: Failed to create UDP socket for route sending");
            return;
        }

        sockaddr_in guiAddr{};
        guiAddr.sin_family = AF_INET;
        guiAddr.sin_port = htons(GUI_PORT);
        inet_pton(AF_INET, GUI_IP, &guiAddr.sin_addr);

        // Отправляем данные
        ssize_t sent = sendto
        (
            sockfd,
            jsonData.c_str(),
            jsonData.size(),
            0,
            (sockaddr*)&guiAddr,
            sizeof(guiAddr)
        );

        if (sent < 0) 
        {
            log(LogLevel::Error, "GUI: Failed to send route data");
        }
        else 
        {
            log(LogLevel::Info, "GUI: Route sent successfully (" +
                std::to_string(keypoints.size()) + " keypoints)");
            log(LogLevel::Debug, "Route data: " + jsonData);
        }

        close(sockfd);
    }
    catch (const std::exception& e) 
    {
        log(LogLevel::Error, "GUI: Error sending route: " + std::string(e.what()));
    }
}

void startCommandListener(Robot& rover, EventQueue& eventQueue)
{
    g_stopListener.store(false, std::memory_order_relaxed);
    g_listenerThread = std::thread(udpListenerLoop, std::ref(rover), std::ref(eventQueue));
}

void stopCommandListener()
{
    g_stopListener.store(true, std::memory_order_relaxed);
    if (g_listenerThread.joinable())
    {
        g_listenerThread.join();
    }
}