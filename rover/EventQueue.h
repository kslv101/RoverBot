// EventQueue.h
#pragma once
#include <queue>
#include <mutex>
#include <optional>
#include <chrono>
#include "Event.h"

// ѕотокобезопасна€ очередь событий (Event-driven architecture)
// 
// ѕринципы работы:
// 1. Ќеблокирующий try_pop() Ч главный поток не зависает
// 2. Ѕлокирующий push() Ч быстра€ операци€, задержка минимальна
// 3. FIFO (первый пришел Ч первый обработан)
// 4. ѕотокобезопасность через std::mutex
class EventQueue
{
public:
    EventQueue() = default;
    ~EventQueue() = default;

    // «апрещаем копирование (чтобы избежать гонок при копировании очереди)
    EventQueue(const EventQueue&) = delete;
    EventQueue& operator=(const EventQueue&) = delete;

    // –азрешаем перемещение (если потребуетс€ передать очередь)
    EventQueue(EventQueue&&) noexcept = default;
    EventQueue& operator=(EventQueue&&) noexcept = default;

    // ƒобавить событие в очередь (потокобезопасно)
    // 
    // ѕараметры:
    //   event - событие, которое будет перемещено в очередь
    // 
    // ѕримечание: метод блокирует мьютекс на врем€ копировани€/перемещени€
    void push(Event event)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        queue_.push(std::move(event));
    }

    // »звлечь событие из очереди (потокобезопасно)
    // 
    // ¬озвращает:
    //   std::nullopt - если очередь пуста (Ќ≈ блокирует вызывающий поток)
    //   Event - следующее событие в очереди (по принципу FIFO)
    // 
    // ѕример использовани€:
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

    // ѕроверить, пуста ли очередь (потокобезопасно)
    // 
    // ¬озвращает:
    //   true - если в очереди нет событий
    //   false - если есть хот€ бы одно событие
    [[nodiscard]] bool empty() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.empty();
    }

    // ѕолучить текущий размер очереди (потокобезопасно)
    // 
    // ¬озвращает:
    //    оличество событий, ожидающих обработки
    [[nodiscard]] size_t size() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.size();
    }

    // ќчистить очередь (потокобезопасно)
    // 
    // ”дал€ет все ожидающие событи€
    void clear()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        while (!queue_.empty())
        {
            queue_.pop();
        }
    }

private:
    // ћьютекс защищает доступ к очереди
    mutable std::mutex mtx_;

    // ќчередь событий (FIFO)
    std::queue<Event> queue_;
};