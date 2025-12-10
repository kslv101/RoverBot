// EventQueue.h
#pragma once
#include <queue>
#include <mutex>
#include <optional>
#include <chrono>
#include "Event.h"

// Потокобезопасная очередь событий (Event-driven architecture)
// 
// Принципы работы:
// 1. Неблокирующий try_pop() — главный поток не зависает
// 2. Блокирующий push() — быстрая операция, задержка минимальна
// 3. FIFO (первый пришел — первый обработан)
// 4. Потокобезопасность через std::mutex
class EventQueue
{
public:
    EventQueue() = default;
    ~EventQueue() = default;

    // Запрещаем копирование (чтобы избежать гонок при копировании очереди)
    EventQueue(const EventQueue&) = delete;
    EventQueue& operator=(const EventQueue&) = delete;

    // Разрешаем перемещение (если потребуется передать очередь)
    EventQueue(EventQueue&&) noexcept = default;
    EventQueue& operator=(EventQueue&&) noexcept = default;

    // Добавить событие в очередь
    // 
    // Параметры:
    //   event - событие, которое будет перемещено в очередь
    // 
    // Примечание: метод блокирует мьютекс на время копирования/перемещения
    void push(Event event)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        queue_.push(std::move(event));
    }

    // Извлечь событие из очереди 
    // 
    // Возвращает:
    //   std::nullopt - если очередь пуста (НЕ блокирует вызывающий поток)
    //   Event - следующее событие в очереди (по принципу FIFO)
    // 
    // Пример использования:
    //   if (auto event = queue.try_pop()) { /* обработать *event */ }
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
    // 
    // Возвращает:
    //   true - если в очереди нет событий
    //   false - если есть хотя бы одно событие
    [[nodiscard]] bool empty() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.empty();
    }

    // Получить текущий размер очереди
    // 
    // Возвращает:
    //   Количество событий, ожидающих обработки
    [[nodiscard]] size_t size() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.size();
    }

    // Очистить очередь
    // 
    // Удаляет все ожидающие события
    void clear()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        while (!queue_.empty())
        {
            queue_.pop();
        }
    }

private:
    // Мьютекс защищает доступ к очереди
    mutable std::mutex mtx_;

    // Очередь событий (FIFO)
    std::queue<Event> queue_;
};