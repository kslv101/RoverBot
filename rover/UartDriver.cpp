#include "UartDriver.h"
#include "Logger.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <errno.h>
#include <system_error>

UartDriver::UartDriver(const std::string& port, int baud_rate)
    : m_port(port), m_baudRate(baud_rate), m_fd(-1), m_connected(false) 
{}

UartDriver::~UartDriver() 
{
    disconnect();
}

bool UartDriver::connect() 
{
    // Открытие последовательного порта
    m_fd = open(m_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (m_fd == -1) 
    {
        log(LogLevel::Error, "UART: Failed to open port " + m_port + ", error: " + strerror(errno));
        return false;
    }

    // Конфигурация порта
    if (!configurePort()) 
    {
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

void UartDriver::disconnect() 
{
    if (m_fd != -1) 
    {
        close(m_fd);
        m_fd = -1;
    }
    m_connected = false;
    log(LogLevel::Info, "UART: Disconnected from " + m_port);
}

bool UartDriver::configurePort() 
{
    struct termios tty;

    // Получение текущих параметров порта
    if (tcgetattr(m_fd, &tty) != 0) 
    {
        log(LogLevel::Error, "UART: Failed to get termios attributes");
        return false;
    }

    // Устанавка скорости передачи (baud rate)
    speed_t baud;
    switch (m_baudRate) 
    {
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

    // Применение настроек
    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) 
    {
        log(LogLevel::Error, "UART: Failed to set termios attributes");
        return false;
    }

    return true;
}

bool UartDriver::sendRaw(const uint8_t* data, size_t size) 
{
    if (!m_connected || m_fd == -1) return false;

    ssize_t bytesWritten = write(m_fd, data, size);
    if (bytesWritten != static_cast<ssize_t>(size)) 
    {
        log(LogLevel::Warn, "UART: Failed to write all bytes");
        return false;
    }

    // Ожидание передачи данных
    tcdrain(m_fd);
    return true;
}

void UartDriver::setPacketCallback(PacketCallback callback) 
{
    m_callback = callback;
}

void UartDriver::processIncomingData() {
    if (!m_connected || m_fd == -1) return;

    uint8_t tempBuffer[1024];
    size_t bytesRead = read(m_fd, tempBuffer, sizeof(tempBuffer));

    if (bytesRead <= 0) return;

    // Добавляем свежепрочитанные данные в конец нашего накопителя
    m_readBuffer.insert(m_readBuffer.end(), tempBuffer, tempBuffer + bytesRead);

    // Пытаемся обработать все полные пакеты, которые сейчас есть в буфере
    size_t i = 0;
    while (i < m_readBuffer.size()) {
        // 1. Ищем маркер начала пакета
        if (m_readBuffer[i] == 0xAA) {

            // Допустим, мы знаем, что ACK всегда 4 байта
            size_t expectedSize = 4;

            // ПРОВЕРКА: Хватает ли нам данных для целого пакета?
            if (i + expectedSize <= m_readBuffer.size()) {
                // Пакет полностью в буфере! Передаем его в callback
                if (m_callback) {
                    m_callback(&m_readBuffer[i], expectedSize);
                }
                i += expectedSize; // Сдвигаем индекс на размер пакета
                continue; // Ищем следующий пакет
            }
            else {
                // Пакет пришел не полностью. Прекращаем обработку, 
                // оставляем данные в буфере до следующего вызова read()
                break;
            }
        }
        else {
            // Если байт не 0xAA, значит это какой-то мусор или текст, просто пропускаем
            i++;
        }
    }

    // Удаляем из буфера всё, что уже обработали
    if (i > 0) {
        m_readBuffer.erase(m_readBuffer.begin(), m_readBuffer.begin() + i);
    }

    // Ограничение размера буфера (на случай критических ошибок, чтобы не съел память)
    if (m_readBuffer.size() > 4096) {
        m_readBuffer.clear();
    }
}

//void UartDriver::processIncomingData() 
//{
//    if (!m_connected || m_fd == -1 || !m_callback) return;
//
//    uint8_t buffer[256];
//    ssize_t bytesRead = read(m_fd, buffer, sizeof(buffer));
//
//    if (bytesRead > 0) 
//    {
//        // Поиск пакетов в данных
//        for (size_t i = 0; i < bytesRead; i++) 
//        {
//            if (buffer[i] == 0xAA && i + sizeof(commands::CommandPacket) <= bytesRead) 
//            {
//                // Найден пакет - вызываем callback
//                m_callback(&buffer[i], sizeof(commands::CommandPacket));
//                i += sizeof(commands::CommandPacket) - 1; // Пропускаем обработанные байты
//            }
//        }
//        log(LogLevel::Debug, "UART: Received " + std::to_string(bytesRead) + " bytes");
//    }
//}