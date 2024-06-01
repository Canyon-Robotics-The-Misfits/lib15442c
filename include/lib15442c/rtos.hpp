#pragma once

#ifndef LIB15442C_MOCK_DEVICES_ONLY
#include "pros/rtos.hpp"
#endif
#include <chrono>

#ifdef LIB15442C_MOCK_DEVICES_ONLY
namespace pros
{
    void delay(int ms)
    {
        // Do nothing
    }
}
#endif

namespace lib15442c
{
#ifndef LIB15442C_MOCK_DEVICES_ONLY
    class Mutex
    {
        pros::Mutex mutex;

    public:
        void lock()
        {
            mutex.lock();
        }
        bool try_lock_for(uint32_t timeout = TIMEOUT_MAX)
        {
            return mutex.try_lock_for(std::chrono::milliseconds(TIMEOUT_MAX));
        }
        void take(uint32_t timeout = TIMEOUT_MAX)
        {
            mutex.take(timeout);
        }
        void unlock()
        {
            mutex.unlock();
        }
    };
#else
    class Mutex
    {
    public:
        void lock(){};
        void take(uint32_t timeout = 0){};
        void unlock(){};
    };
#endif
}