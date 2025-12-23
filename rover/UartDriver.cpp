// UartDriver.cpp
#include "UartDriver.h"
#include "Logger.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <errno.h>
#include <system_error>

UartDriver::UartDriver(const std::string& port, int baud_rate)
    : m_port(port), m_baudRate(baud_rate), m_fd(-1), m_connected(false) {
}

UartDriver::~UartDriver() {
    disconnect();
}

bool UartDriver::connect() {
    // Открываем последовательный порт
    m_fd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_fd == -1) {
        log(LogLevel::Error, "UART: Failed to open port " + m_port +
            ", error: " + strerror(errno));
        return false;
    }

    // Конфигурация порта
    if (!configurePort()) {
        close(m_fd);
        m_fd = -1;
        return false;
    }

    // Очистка входящего и исходящего буферов
    tcflush(m_fd, TCIOFLUSH);

    m_connected = true;
    log(LogLevel::Info, "UART: Connected to " + m_port);
    return true;
}

void UartDriver::disconnect() {
    if (m_fd != -1) {
        close(m_fd);
        m_fd = -1;
    }
    m_connected = false;
    log(LogLevel::Info, "UART: Disconnected from " + m_port);
}

bool UartDriver::configurePort() {
    struct termios tty;

    // Получаем текущие параметры порта
    if (tcgetattr(m_fd, &tty) != 0) {
        log(LogLevel::Error, "UART: Failed to get termios attributes");
        return false;
    }

    // Устанавливаем скорость передачи (baud rate)
    speed_t baud;
    switch (m_baudRate) {
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    default:
        log(LogLevel::Error, "UART: Unsupported baud rate: " + std::to_string(m_baudRate));
        return false;
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // Настройка параметров фрейма: 8N1 (8 бит данных, без контроля четности, 1 стоп-бит)
    tty.c_cflag &= ~PARENB;        // Без контроля четности
    tty.c_cflag &= ~CSTOPB;        // 1 стоп-бит
    tty.c_cflag &= ~CSIZE;         // Сбрасываем размер байта
    tty.c_cflag |= CS8;            // 8 бит данных
    tty.c_cflag &= ~CRTSCTS;       // Без аппаратного контроля потока (RTS/CTS)

    // Настройка режима работы
    tty.c_cflag |= CREAD | CLOCAL; // Включаем приемник, игнорируем управляющие линии

    // Настройка канонического/неканонического режима
    tty.c_lflag &= ~ICANON;        // Неканонический режим (сырой ввод)
    tty.c_lflag &= ~ECHO;          // Без эха
    tty.c_lflag &= ~ECHOE;         // Без эха удаления символов
    tty.c_lflag &= ~ECHONL;        // Без эха новой строки
    tty.c_lflag &= ~ISIG;          // Без обработки сигналов

    // Настройка обработки входных данных
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Без программного контроля потока
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Без специальной обработки байтов

    // Настройка выходных данных
    tty.c_oflag &= ~OPOST;         // Без специальной обработки выходных данных
    tty.c_oflag &= ~ONLCR;         // Без замены \n на \r\n

    // Настройка таймаутов
    tty.c_cc[VTIME] = 10;          // Таймаут в децисекундах (1 секунда = 10)
    tty.c_cc[VMIN] = 0;           // Минимальное количество байт для чтения (0 = не блокировать)

    // Применяем настройки
    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
        log(LogLevel::Error, "UART: Failed to set termios attributes");
        return false;
    }

    return true;
}

bool UartDriver::sendRaw(const uint8_t* data, size_t size) {
    if (!m_connected || m_fd == -1) return false;

    ssize_t bytesWritten = write(m_fd, data, size);
    if (bytesWritten != static_cast<ssize_t>(size)) {
        log(LogLevel::Warn, "UART: Failed to write all bytes");
        return false;
    }

    // Ожидание передачи данных
    tcdrain(m_fd);
    return true;
}

void UartDriver::setPacketCallback(PacketCallback callback) {
    m_callback = callback;
}

void UartDriver::processIncomingData() {
    if (!m_connected || m_fd == -1 || !m_callback) return;

    uint8_t buffer[256];
    ssize_t bytesRead = read(m_fd, buffer, sizeof(buffer));

    if (bytesRead > 0) {
        // Поиск пакетов в данных
        for (size_t i = 0; i < bytesRead; i++) {
            if (buffer[i] == 0xAA && i + sizeof(commands::CommandPacket) <= bytesRead) {
                // Найден пакет - вызываем callback
                m_callback(&buffer[i], sizeof(commands::CommandPacket));
                i += sizeof(commands::CommandPacket) - 1; // Пропускаем обработанные байты
            }
        }
        log(LogLevel::Debug, "UART: Received " + std::to_string(bytesRead) + " bytes");
    }
}
//// UartDriver.cpp
//#include "UartDriver.h"
//#include "Logger.h"
//#include <windows.h>
//#include <cstring>
//
//UartDriver::UartDriver(const std::string& port, int baud_rate)
//    : m_port(port), m_baudRate(baud_rate), m_handle(INVALID_HANDLE_VALUE), m_connected(false) {
//}
//
//UartDriver::~UartDriver() {
//    disconnect();
//}
//
//bool UartDriver::connect() {
//    std::string fullPort = m_port;
//    if (m_port.find("COM") != std::string::npos && m_port.length() > 4) {
//        fullPort = "\\\\.\\" + m_port;
//    }
//
//    m_handle = CreateFileA(
//        fullPort.c_str(),
//        GENERIC_READ | GENERIC_WRITE,
//        0,
//        NULL,
//        OPEN_EXISTING,
//        FILE_ATTRIBUTE_NORMAL,
//        NULL
//    );
//
//    if (m_handle == INVALID_HANDLE_VALUE) {
//        DWORD error = GetLastError();
//        log(LogLevel::Error, "UART: Failed to open port " + m_port +
//            ", error: " + std::to_string(error));
//        return false;
//    }
//
//    DCB dcb = { 0 };
//    dcb.DCBlength = sizeof(dcb);
//    if (!GetCommState((HANDLE)m_handle, &dcb)) {
//        log(LogLevel::Error, "UART: Failed to get comm state");
//        CloseHandle((HANDLE)m_handle);
//        return false;
//    }
//
//    dcb.BaudRate = m_baudRate;
//    dcb.ByteSize = 8;
//    dcb.StopBits = ONESTOPBIT;
//    dcb.Parity = NOPARITY;
//    dcb.fDtrControl = DTR_CONTROL_ENABLE;
//
//    if (!SetCommState((HANDLE)m_handle, &dcb)) {
//        log(LogLevel::Error, "UART: Failed to set comm state");
//        CloseHandle((HANDLE)m_handle);
//        return false;
//    }
//
//    COMMTIMEOUTS timeouts = { 0 };
//    timeouts.ReadIntervalTimeout = 50;
//    timeouts.ReadTotalTimeoutConstant = 100;
//    timeouts.WriteTotalTimeoutConstant = 100;
//    SetCommTimeouts((HANDLE)m_handle, &timeouts);
//
//    PurgeComm((HANDLE)m_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
//    m_connected = true;
//
//    log(LogLevel::Info, "UART: Connected to " + m_port);
//    return true;
//}
//
//void UartDriver::disconnect() {
//    if (m_handle != INVALID_HANDLE_VALUE) {
//        CloseHandle((HANDLE)m_handle);
//        m_handle = INVALID_HANDLE_VALUE;
//    }
//    m_connected = false;
//    log(LogLevel::Info, "UART: Disconnected from " + m_port);
//}
//
//bool UartDriver::sendRaw(const uint8_t* data, size_t size) {
//    if (!m_connected || m_handle == INVALID_HANDLE_VALUE) return false;
//
//    DWORD bytesWritten;
//    return WriteFile(m_handle, data, static_cast<DWORD>(size), &bytesWritten, NULL) &&
//        bytesWritten == size;
//}
//
//void UartDriver::setPacketCallback(PacketCallback callback) {
//    m_callback = callback;
//}
//
//void UartDriver::processIncomingData() {
//    if (!m_connected || m_handle == INVALID_HANDLE_VALUE || !m_callback) return;
//
//    uint8_t buffer[256];
//    DWORD bytesRead;
//
//    if (ReadFile(m_handle, buffer, sizeof(buffer), &bytesRead, NULL) && bytesRead > 0) {
//        // Ищем пакеты в буфере
//        for (size_t i = 0; i < bytesRead; i++) {
//            if (buffer[i] == 0xAA && i + sizeof(commands::commands::CommandPacket) <= bytesRead) {
//                // Найден возможный пакет
//                m_callback(&buffer[i], sizeof(commands::commands::CommandPacket));
//                i += sizeof(commands::commands::CommandPacket) - 1; // Пропускаем обработанный пакет
//            }
//        }
//    }
//    // ЛОГИРУЕМ ПОЛУЧЕННЫЕ ДАННЫЕ ДЛЯ ОТЛАДКИ
//    log(LogLevel::Debug, "UART: Received " + std::to_string(bytesRead) + " bytes");
//}