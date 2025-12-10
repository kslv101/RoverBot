// UartDriver.cpp
#include "UartDriver.h"
#include "Logger.h"
#include <windows.h>
#include <cstring>

UartDriver::UartDriver(const std::string& port, int baud_rate)
    : m_port(port), m_baudRate(baud_rate), m_handle(INVALID_HANDLE_VALUE), m_connected(false) {
}

UartDriver::~UartDriver() {
    disconnect();
}

bool UartDriver::connect() {
    std::string fullPort = m_port;
    if (m_port.find("COM") != std::string::npos && m_port.length() > 4) {
        fullPort = "\\\\.\\" + m_port;
    }

    m_handle = CreateFileA(
        fullPort.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    if (m_handle == INVALID_HANDLE_VALUE) {
        DWORD error = GetLastError();
        log(LogLevel::Error, "UART: Failed to open port " + m_port +
            ", error: " + std::to_string(error));
        return false;
    }

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState((HANDLE)m_handle, &dcb)) {
        log(LogLevel::Error, "UART: Failed to get comm state");
        CloseHandle((HANDLE)m_handle);
        return false;
    }

    dcb.BaudRate = m_baudRate;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;

    if (!SetCommState((HANDLE)m_handle, &dcb)) {
        log(LogLevel::Error, "UART: Failed to set comm state");
        CloseHandle((HANDLE)m_handle);
        return false;
    }

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 100;
    timeouts.WriteTotalTimeoutConstant = 100;
    SetCommTimeouts((HANDLE)m_handle, &timeouts);

    PurgeComm((HANDLE)m_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    m_connected = true;

    log(LogLevel::Info, "UART: Connected to " + m_port);
    return true;
}

void UartDriver::disconnect() {
    if (m_handle != INVALID_HANDLE_VALUE) {
        CloseHandle((HANDLE)m_handle);
        m_handle = INVALID_HANDLE_VALUE;
    }
    m_connected = false;
    log(LogLevel::Info, "UART: Disconnected from " + m_port);
}

bool UartDriver::sendRaw(const uint8_t* data, size_t size) {
    if (!m_connected || m_handle == INVALID_HANDLE_VALUE) return false;

    DWORD bytesWritten;
    return WriteFile(m_handle, data, static_cast<DWORD>(size), &bytesWritten, NULL) &&
        bytesWritten == size;
}

void UartDriver::setPacketCallback(PacketCallback callback) {
    m_callback = callback;
}

void UartDriver::processIncomingData() {
    if (!m_connected || m_handle == INVALID_HANDLE_VALUE || !m_callback) return;

    uint8_t buffer[256];
    DWORD bytesRead;

    if (ReadFile(m_handle, buffer, sizeof(buffer), &bytesRead, NULL) && bytesRead > 0) {
        // Ищем пакеты в буфере
        for (size_t i = 0; i < bytesRead; i++) {
            if (buffer[i] == 0xAA && i + sizeof(commands::CommandPacket) <= bytesRead) {
                // Найден возможный пакет
                m_callback(&buffer[i], sizeof(commands::CommandPacket));
                i += sizeof(commands::CommandPacket) - 1; // Пропускаем обработанный пакет
            }
        }
    }
    // ЛОГИРУЕМ ПОЛУЧЕННЫЕ ДАННЫЕ ДЛЯ ОТЛАДКИ
    log(LogLevel::Debug, "UART: Received " + std::to_string(bytesRead) + " bytes");
}