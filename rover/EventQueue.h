#pragma once
#include <queue>
#include <mutex>
#include <optional>
#include <chrono>
#include "Event.h"

class EventQueue
{
public:
    EventQueue() = default;
    ~EventQueue() = default;

    // Запрет копирования
    EventQueue(const EventQueue&) = delete;
    EventQueue& operator=(const EventQueue&) = delete;

    // Разрешение перемещения
    EventQueue(EventQueue&&) noexcept = default;
    EventQueue& operator=(EventQueue&&) noexcept = default;

    void push(Event event)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        queue_.push(std::move(event));
    }

    // Извлечь событие из очереди 
    [[nodiscard]] std::optional<Event> try_pop()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.empty())
        {
            return std::nullopt;
        }
        Event ev = std::move(queue_.front());
        queue_.pop();
        return ev;
    }

    // Проверить, пуста ли очередь
    [[nodiscard]] bool empty() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.empty();
    }

    // Получить текущий размер очереди
    [[nodiscard]] size_t size() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.size();
    }

    // Очистить очередь
    void clear()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        while (!queue_.empty())
        {
            queue_.pop();
        }
    }

private:
    mutable std::mutex mtx_;

    // Очередь событий (FIFO)
    std::queue<Event> queue_;
};