#pragma once

#include <deque>
#include <mutex>
#include <condition_variable>

namespace core
{
    template <typename MsgType>
    struct ThreadSafeDeque
    {
        std::deque<MsgType> deque;
        std::mutex mtx;
        std::condition_variable cv;
    };
}
