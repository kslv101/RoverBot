// EventQueue.h
#pragma once
#include <queue>
#include <mutex>
#include "Event.h"

class EventQueue
{
public:
    void push(EventType event)
    {
        std::lock_guard lock(mtx_);
        queue_.push(event);
    }

    std::optional<EventType> pop()
    {
        std::lock_guard lock(mtx_);
        if (queue_.empty()) return std::nullopt;
        auto ev = queue_.front();
        queue_.pop();
        return ev;
    }

    bool empty() const
    {
        std::lock_guard lock(mtx_);
        return queue_.empty();
    }

private:
    mutable std::mutex mtx_;
    std::queue<EventType> queue_;
};