#pragma once

#include <thread>
#include <condition_variable>

class LoopLock
    {
    public:
        void lock()
        {
            mutex_.lock();
        }

        void unlock()
        {
            mutex_.unlock();
        }

        void lockInterruptCtx()
        {
            lock();
        }

        void unlockInterruptCtx()
        {
            unlock();
        }
    private:
        std::recursive_mutex mutex_;
    };

    class EventCondition
    {
    public:
        EventCondition() : notified_(false) {}

        template <typename TLock>
        void wait(TLock& lock)
        {
            if (!notified_) {
                cond_.wait(lock);
            }
            notified_ = false;
        }

        void notify()
        {
            notified_ = true;
            cond_.notify_all();
        }

    private:
        std::condition_variable_any cond_;
        bool notified_;
    };

