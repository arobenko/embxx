//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

// This library is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/// @file embxx/util/EventLoop.h
/// Contains EventLoop class definition.

#pragma once

#include <cstddef>
#include <type_traits>
#include <mutex>
#include <new>
#include <functional>

#include "embxx/container/StaticQueue.h"
#include "embxx/util/ScopeGuard.h"

namespace embxx
{

namespace util
{

/// @addtogroup util
/// @{

/// @brief Implements basic event loop for bare metal platform.
/// @details Provides an ability to post new handlers to be executed in
///          non-interrupt context.
/// @tparam TSize Size in bytes to be allocated as data member for handlers
///         registration. It cannot be changed afterwards.
/// @tparam TLock "Lockable" class. It must provide the following functions:
///         @li @code void lock(); @endcode
///         @li @code void unlock(); @endcode
///         @li @code void lockInterruptCtx(); @endcode
///         @li @code void unlockInterruptCtx(); @endcode
///
///         This lock must have a default constructor. It is used to
///         protect update of the pending handlers queue.
/// @tparam TCond Wait condition variable class. It must have a default
///         constructor and provide the following functions:
///         @li @code template <typename TLock> void wait(TLock& lock); @endcode
///         @li @code void notify(); @endcode
///
///         Both of these functions are called after call to lock() member
///         function of the TLock object.
/// @headerfile embxx/util/EventLoop.h
template <std::size_t TSize,
          typename TLock,
          typename TCond>
class EventLoop
{
public:
    /// @brief Type of the lock
    typedef TLock LockType;

    /// @brief Type of the condition variable
    typedef TCond CondType;

    /// @brief Constructor.
    EventLoop();

    /// @brief Destructor
    ~EventLoop() = default;

    /// @brief Get reference to the lock.
    LockType& getLock();

    /// @brief Get reference to the condition variable
    CondType& getCond();

    /// @brief Post new handler for execution.
    /// @details Acquires regular context lock. The task is added to the
    ///          execution queue. If the execution queue is empty before the
    ///          new handler is added, the condition variable is signalled by
    ///          calling its notify() member function.
    /// @param[in] task R-value reference to new handler functor.
    /// @return true in case the handler was successfully posted, false if
    ///         there is not enough space in the execution queue.
    /// @note Thread safety: Safe
    /// @note Exception guarantee: Basic
    template <typename TTask>
    bool post(TTask&& task);

    /// @brief Post new handler for execution from interrupt context.
    /// @details Acquires interrupt context lock. The task is added to the
    ///          execution queue. If the execution queue is empty before the
    ///          new handler is added, the condition variable is signalled by
    ///          calling its notify() member function.
    /// @param[in] task R-value reference to new handler functor.
    /// @return true in case the handler was successfully posted, false if
    ///         there is not enough space in the execution queue.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    template <typename TTask>
    bool postInterruptCtx(TTask&& task);

    /// @brief Event loop execution function.
    /// @details The function keeps executing posted handlers until none
    ///          are left. When execution queue becomes empty the wait(...)
    ///          member function of the condition variable gets called to
    ///          execute blocking wait for new handlers. When new handler
    ///          is added, the condition variable will be signalled and blocking
    ///          wait is expected to be terminated to continue execution of
    ///          the event loop. This function never exits unless stop() was
    ///          called to terminate the execution. After stopping the main
    ///          loop, use reset() member function to enable the loop to be
    ///          executed again.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    void run();

    /// @brief Stop execution of the event loop.
    /// @details The execution may not be stopped immediately. If there is an
    ///          event handler being executed, the loop will be stopped after
    ///          the execution of the handler is finished.
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    void stop();

    /// @brief Reset the state of the event loop.
    /// @details Clear the queue of registered event handlers and resets the
    ///          "stopped" flag to allow new event loop execution.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    void reset();

    /// @brief Perform busy wait.
    /// @details Executes busy wait while allowing other event handlers posted
    ///          by interrupt handlers being processed.
    /// @tparam TPred Predicate class type, must define
    ///         @code bool operator()(); @endcode
    ///         that return true in case busy wait must be terminated.
    /// @tparam TFunc Functor class that will be executed when wait is complete.
    ///         It must define
    ///         @code void operator()(); @endcode
    /// @param pred Any type of reference to predicate object
    /// @param func Any type of reference to "wait complete" function.
    /// @pre The event loop must have enough space to repost the call to
    ///      busyWait. Note that there is no way to notify the caller if post
    ///      operation fails. In debug compilation mode there will be
    ///      an assertion failure in case call to post() returned false, in
    ///      release compilation mode the failure will be silent.
    template <typename TPred, typename TFunc>
    void busyWait(TPred&& pred, TFunc&& func);

private:

    /// @cond DOCUMENT_EVENT_LOOP_TASK
    class Task
    {
    public:
        virtual ~Task();
        virtual std::size_t getSize() const;
        virtual void exec();
    };

    template <typename TTask>
    class TaskBound : public Task
    {

    public:
        explicit TaskBound(const TTask& task);
        explicit TaskBound(TTask&& task);
        virtual ~TaskBound();

        virtual std::size_t getSize() const;
        virtual void exec();

        static const std::size_t Size =
            ((sizeof(TaskBound<typename std::decay<TTask>::type>) - 1) / sizeof(Task)) + 1;

    private:
        TTask task_;
    };
    /// @endcond

    /// @cond DOCUMENT_INTERRUPT_LOCK_WRAPPER
    template <typename TInternalLock>
    class InterruptLockWrapper
    {
    public:
        InterruptLockWrapper(TInternalLock& intLock) : lock_(intLock) {}
        void lock()
        {
            lock_.lockInterruptCtx();
        }

        void unlock()
        {
            lock_.unlockInterruptCtx();
        }
    private:
        TInternalLock& lock_;
    };
    /// @endcond

    typedef typename
        std::aligned_storage<
            sizeof(Task),
            std::alignment_of<Task>::value
        >::type ArrayElemType;

    static const std::size_t ArraySize = TSize / sizeof(Task);
    typedef embxx::container::StaticQueue<ArrayElemType, ArraySize> EventQueue;

    template <typename TTask>
    bool postNoLock(TTask&& task);

    ArrayElemType* getAllocPlace(std::size_t requiredQueueSize);

    EventQueue queue_;
    LockType lock_;
    CondType cond_;
    volatile bool stopped_;
};

/// @}

// Implementation
template <std::size_t TSize,
          typename TLock,
          typename TCond>
EventLoop<TSize, TLock, TCond>::EventLoop()
    : stopped_(false)
{
    GASSERT(queue_.isEmpty());
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
typename EventLoop<TSize, TLock, TCond>::LockType&
EventLoop<TSize, TLock, TCond>::getLock()
{
    return lock_;
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
typename EventLoop<TSize, TLock, TCond>::CondType&
EventLoop<TSize, TLock, TCond>::getCond()
{
    return cond_;
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
template <typename TTask>
bool EventLoop<TSize, TLock, TCond>::post(TTask&& task)
{
    std::lock_guard<LockType> guard(lock_);
    return postNoLock(std::forward<TTask>(task));
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
template <typename TTask>
bool EventLoop<TSize, TLock, TCond>::postInterruptCtx(
    TTask&& task)
{
    InterruptLockWrapper<LockType> wrapperLock(lock_);
    std::lock_guard<decltype(wrapperLock)> guard(wrapperLock);
    return postNoLock(std::forward<TTask>(task));
}


template <std::size_t TSize,
          typename TLock,
          typename TCond>
void EventLoop<TSize, TLock, TCond>::run()
{
    while (true) {
        lock_.lock();
        auto lockGuard = embxx::util::makeScopeGuard(
            [this]()
            {
                lock_.unlock();
            });

        while (!stopped_) {
            volatile bool empty = queue_.isEmpty();
            if (empty) {
                break;
            }

            auto taskPtr = reinterpret_cast<Task*>(&queue_.front());
            auto sizeToRemove = taskPtr->getSize();
            lock_.unlock();
            taskPtr->exec();
            taskPtr->~Task();
            lock_.lock();
            queue_.popFront(sizeToRemove);
        }

        if (stopped_) {
            break;
        }

        // Still locked prior to wait
        cond_.wait(lock_);
    }
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
void EventLoop<TSize, TLock, TCond>::stop()
{
    std::lock_guard<LockType> guard(lock_);
    stopped_ = true;
    cond_.notify();
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
void EventLoop<TSize, TLock, TCond>::reset()
{
    std::lock_guard<LockType> guard(lock_);
    stopped_ = false;
    queue_.clear();
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
template <typename TPred, typename TFunc>
void EventLoop<TSize, TLock, TCond>::busyWait(TPred&& pred, TFunc&& func)
{
    if (pred()) {
        bool result = post(std::forward<TFunc>(func));
        GASSERT(result);
        static_cast<void>(result);
        return;
    }

    bool result = post(
        [this, pred, func]()
        {
            busyWait(std::move(pred), std::move(func));
        });
    GASSERT(result);
    static_cast<void>(result);
}

/// @cond DOCUMENT_EVENT_LOOP_TASK
template <std::size_t TSize,
          typename TLock,
          typename TCond>
EventLoop<TSize, TLock, TCond>::Task::~Task()
{
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
std::size_t EventLoop<TSize, TLock, TCond>::Task::getSize() const
{
    return 1;
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
void EventLoop<TSize, TLock, TCond>::Task::exec()
{
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
template <typename TTask>
EventLoop<TSize, TLock, TCond>::TaskBound<TTask>::TaskBound(const TTask& task)
    : task_(task)
{
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
template <typename TTask>
EventLoop<TSize, TLock, TCond>::TaskBound<TTask>::TaskBound(TTask&& task)
    : task_(std::move(task))
{
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
template <typename TTask>
EventLoop<TSize, TLock, TCond>::TaskBound<TTask>::~TaskBound()
{
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
template <typename TTask>
std::size_t EventLoop<TSize, TLock, TCond>::TaskBound<TTask>::getSize() const
{
    return Size;
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
template <typename TTask>
void EventLoop<TSize, TLock, TCond>::TaskBound<TTask>::exec()
{
    task_();
}

/// @endcond

template <std::size_t TSize,
          typename TLock,
          typename TCond>
template <typename TTask>
bool EventLoop<TSize, TLock, TCond>::postNoLock(TTask&& task)
{
    typedef TaskBound<typename std::decay<TTask>::type> TaskBoundType;
    static_assert(std::alignment_of<Task>::value == std::alignment_of<TaskBoundType>::value,
        "Alignment of TaskBound must be same as alignment of Task");

    static const std::size_t requiredQueueSize = TaskBoundType::Size;

    bool wasEmpty = queue_.isEmpty();

    auto placePtr = getAllocPlace(requiredQueueSize);
    if (placePtr == nullptr) {
        return false;
    }

    auto taskPtr = new (placePtr) TaskBoundType(std::forward<TTask>(task));
    static_cast<void>(taskPtr);

    GASSERT(!queue_.isEmpty());
    GASSERT(requiredQueueSize <= queue_.size());

    if (wasEmpty) {
        cond_.notify();
    }

    return true;
}

template <std::size_t TSize,
          typename TLock,
          typename TCond>
typename EventLoop<TSize, TLock, TCond>::ArrayElemType*
EventLoop<TSize, TLock, TCond>::getAllocPlace(
    std::size_t requiredQueueSize)
{
    auto invalidIter = queue_.invalidIter();
    while (true)
    {
        if ((queue_.capacity() - queue_.size()) < requiredQueueSize) {
            return nullptr;
        }

        auto curSize = queue_.size();
        if (queue_.isLinearised()) {
            auto dist =
                static_cast<std::size_t>(
                    std::distance(queue_.arrayTwo().second, invalidIter));
            if ((0 < dist) && (dist < requiredQueueSize)) {
                queue_.resize(curSize + 1);
                auto placePtr = static_cast<void*>(&queue_.back());
                auto taskPtr = new (placePtr) Task();
                static_cast<void>(taskPtr);
                continue;
            }
        }

        queue_.resize(curSize + requiredQueueSize);
        return &queue_[curSize];
    }
}

}  // namespace util

}  // namespace embxx
