//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

// This file is free software: you can redistribute it and/or modify
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

/// @file embxx/io/WriteQueue.h
/// Contains WriteQueue class.

#pragma once

#include <cstddef>
#include <limits>
#include <algorithm>
#include <functional>

#include "embxx/error/ErrorStatus.h"
#include "embxx/util/StaticFunction.h"
#include "embxx/container/StaticQueue.h"

namespace embxx
{

namespace io
{

/// @addtogroup io
/// @{

/// @brief Queue for independent write operations.
/// @details This class is designed to wrap a driver that supports only single
///          write request at a time. It creates a queue for multiple
///          independent write requests and forwards them into the driver
///          when necessary.
/// @tparam TDriver Driver class that provides the following interface:
///         @code
///         // Asynchronous write function. "func" callback function must have
///         // "void (const embxx::error::ErrorStatus&, std::size_t)" signature.
///         template <typename TFunc>
///         void asyncWrite(const CharType* buf, std::size_t size, TFunc&& func);
///
///         // Cancel current write operation
///         bool cancelWrite();
///         @endcode
///         See embxx::driver::Character for reference.
/// @tparam TSize Maximal size of the queue in terms of number of outstanding
///         write requests.
/// @tparam THandler Handler class. Must be either std::function or
///         embxx::util::StaticFunction and have
///         "void (const embxx::error::ErrorStatus&, std::size_t)" signature.
/// @headerfile embxx/io/WriteQueue.h
template <typename TDriver,
          std::size_t TSize,
          typename THandler = util::StaticFunction<void (const embxx::error::ErrorStatus&, std::size_t)> >
class WriteQueue
{
public:
    /// @brief Driver type
    typedef TDriver Driver;

    /// @brief Size of the queue
    static const std::size_t Size = TSize;

    /// @brief Write request handler type
    typedef THandler Handler;

    /// @brief Character type
    typedef typename Driver::CharType CharType;

    /// @brief Handle type returned by call to asyncWrite()
    typedef unsigned WriteHandle;

    /// @brief Definition of invalid handle returned by asyncWrite() in
    ///        case of an error.
    static const WriteHandle InvalidWriteHandle =
        std::numeric_limits<WriteHandle>::max();


    /// @brief Constructor
    /// @param driv Reference to driver object
    WriteQueue(Driver& driv);

    /// @brief Copy constructor is deleted
    WriteQueue(const WriteQueue&) = delete;

    /// @brief Move constructor is deleted
    WriteQueue(WriteQueue&&) = delete;

    /// @brief Destructor
    ~WriteQueue();

    /// @brief Copy assignment is deleted
    WriteQueue& operator=(const WriteQueue&) = delete;

    /// @brief Move assignment is deleted
    WriteQueue& operator=(WriteQueue&&) = delete;

    /// @brief Get reference to driver object.
    Driver& driver();

    /// @brief Asynchronous write request
    /// @details The function returns immediately. The callback will be called
    ///          with operation results only when all the data from provided
    ///          buffer has been sent or operation has been cancelled.
    /// @param buf Pointer to the read-only buffer. The buffer mustn't be
    ///            changed or destructed until the callback has been called.
    /// @param size Size of the buffer.
    /// @param func Callback functor that must have following signature:
    ///        @code void callback(const embxx::error::ErrorStatus& status, std::size_t bytesWritten); @endcode
    /// @return Handle of the write operation which may be used to cancel the
    ///         request in the future. If the queue is full prior to request
    ///         InvalidWriteHandle will be returned and it is the responsibility
    ///         of the caller to check whether the write operation was scheduled.
    /// @pre The provided buffer must stay valid and unused until the provided
    ///      callback function is called.
    template <typename TFunc>
    WriteHandle asyncWrite(
        const CharType* buf,
        std::size_t size,
        TFunc&& func);

    /// @brief Asynchronous write request.
    /// @details Similar to previous asyncWrite() but doesn't require callback
    ///          function to be provided. It should be used when the caller
    ///          has no interest in the notification when the write is complete.
    ///          Must be used with care and preferably on constant static
    ///          buffers because the same precondition of not changing the
    ///          provided buffer until the write is complete holds.
    WriteHandle asyncWrite(
        const CharType* buf,
        std::size_t size);

    /// @brief Cancel the write request.
    /// @param handle Write request handle returned by asyncWrite().
    /// @return true in case the request was successfully cancelled and the
    ///         callback will be invoked with embxx::error::ErrorCode::Aborted
    ///         as the status value of operation completion, false otherwise.
    bool cancelWrite(WriteHandle handle);

    /// @brief Cancel all the outstanding write requests.
    void cancelAllWrites();

private:
    struct Node
    {
        template <typename TFunc>
        Node(
            const CharType* start,
            std::size_t size,
            WriteHandle handleId,
            TFunc&& func)
            : start_(start),
              size_(size),
              handleId_(handleId),
              handler_(std::forward<TFunc>(func))
        {
        }

        const CharType* start_;
        std::size_t size_;
        WriteHandle handleId_;
        Handler handler_;
    };

    typedef embxx::container::StaticQueue<Node, Size> Queue;

    void scheduleNewWrite();
    void findAndCleanCancelledWrite();
    bool invokeHandler(
        Node& node,
        const embxx::error::ErrorStatus& status,
        std::size_t bytesWritten);


    Driver& driver_;
    Queue queue_;
    WriteHandle nextHandleId_;
};

/// @}

// Implementation
template <typename TDriver,
          std::size_t TSize,
          typename THandler>
WriteQueue<TDriver, TSize, THandler>::WriteQueue(Driver& driv)
    : driver_(driv),
      nextHandleId_(0)
{
}

template <typename TDriver,
          std::size_t TSize,
          typename THandler>
WriteQueue<TDriver, TSize, THandler>::~WriteQueue()
{
}

template <typename TDriver,
          std::size_t TSize,
          typename THandler>
typename WriteQueue<TDriver, TSize, THandler>::Driver&
WriteQueue<TDriver, TSize, THandler>::driver()
{
    return driver_;
}

template <typename TDriver,
          std::size_t TSize,
          typename THandler>
template <typename TFunc>
typename WriteQueue<TDriver, TSize, THandler>::WriteHandle
WriteQueue<TDriver, TSize, THandler>::asyncWrite(
    const CharType* buf,
    std::size_t size,
    TFunc&& func)
{
    if (queue_.isFull()) {
        findAndCleanCancelledWrite();
        if (queue_.isFull()) {
            return InvalidWriteHandle;
        }
    }

    bool wasEmpty = queue_.isEmpty();
    ++nextHandleId_;
    queue_.pushBack(Node(buf, size, nextHandleId_, std::forward<TFunc>(func)));
    if (wasEmpty) {
        scheduleNewWrite();
    }

    GASSERT(!queue_.isEmpty());
    return nextHandleId_;
}

template <typename TDriver,
          std::size_t TSize,
          typename THandler>
typename WriteQueue<TDriver, TSize, THandler>::WriteHandle
WriteQueue<TDriver, TSize, THandler>::asyncWrite(
    const CharType* buf,
    std::size_t size)
{
    return asyncWrite(buf, size,
        [](const embxx::error::ErrorStatus& es, std::size_t bytesWritten)
        {
            static_cast<void>(es);
            static_cast<void>(bytesWritten);
        });
}

template <typename TDriver,
          std::size_t TSize,
          typename THandler>
bool WriteQueue<TDriver, TSize, THandler>::cancelWrite(WriteHandle handle)
{
    if (queue_.isEmpty()) {
        return false;
    }

    auto findNodeFunc = [handle](typename Queue::LinearisedIteratorRange range) -> typename Queue::LinearisedIterator
        {
            return std::find_if(
                range.first,
                range.second,
                [handle](const Node& node) -> bool
                {
                    return (handle == node.handleId_);
                });
        };

    auto arrayOne = queue_.arrayOne();
    auto iter = findNodeFunc(arrayOne);
    if (iter == arrayOne.first) {
        return driver_.cancelWrite();
    }

    if (iter != arrayOne.second) {
        return invokeHandler(*iter, embxx::error::ErrorCode::Aborted, 0);
    }

    auto arrayTwo = queue_.arrayTwo();
    iter = findNodeFunc(arrayTwo);
    if (iter == arrayTwo.second) {
        return false;
    }

    return invokeHandler(*iter, embxx::error::ErrorCode::Aborted, 0);
}

template <typename TDriver,
          std::size_t TSize,
          typename THandler>
void WriteQueue<TDriver, TSize, THandler>::cancelAllWrites()
{
    if (queue_.isEmpty()) {
        return false;
    }

    return driver_.cancelWrite();

    auto cancelAllFunc =
        [this](typename Queue::LinearisedIterator iterBegin, typename Queue::LinearisedIterator iterEnd)
        {
            std::for_each(iterBegin, iterEnd,
                [this](Node& node)
                {
                    auto result =
                        invokeHandler(node, embxx::error::ErrorCode::Aborted, 0);
                    static_cast<void>(result); // don't care
                });
        };

    auto arrayOne = queue_.arrayOne();
    cancelAllFunc(arrayOne.first + 1, arrayOne.second); // exclude first node
    auto arrayTwo = queue_.arrayTwo();
    cancelAllFunc(arrayTwo.first, arrayTwo.second);
}

template <typename TDriver,
          std::size_t TSize,
          typename THandler>
void WriteQueue<TDriver, TSize, THandler>::scheduleNewWrite()
{
    if (queue_.isEmpty()) {
        return;
    }

    while (true) {
        auto& nextWait = queue_.front();
        if (!nextWait.handler_) {
            queue_.popFront();
            continue;
        }

        driver_.asyncWrite(
            nextWait.start_,
            nextWait.size_,
            [this](const embxx::error::ErrorStatus& es, std::size_t bytesWritten)
            {
                if (!queue_.isEmpty()) {
                    auto result =
                        invokeHandler(queue_.front(), es, bytesWritten);
                    static_cast<void>(result);
                    queue_.popFront();
                    scheduleNewWrite();
                }
            });

        break;
    }
}

template <typename TDriver,
          std::size_t TSize,
          typename THandler>
void WriteQueue<TDriver, TSize, THandler>::findAndCleanCancelledWrite()
{
    auto func = [this](typename Queue::LinearisedIteratorRange range) -> bool
        {
            for (auto iter = range.first; iter != range.second; ++iter) {
                if (!iter->handler_) {
                    queue_.erase(iter);
                    return true;
                }
            }
            return false;
        };

    if (func(queue_.arrayOne())) {
        GASSERT(!queue_.isFull());
        return;
    }

    auto result = func(queue_.arrayTwo());
    static_cast<void>(result);
    GASSERT((!result) || (!queue_.isFull()));
}

template <typename TDriver,
          std::size_t TSize,
          typename THandler>
bool WriteQueue<TDriver, TSize, THandler>::invokeHandler(
    Node& node,
    const embxx::error::ErrorStatus& status,
    std::size_t bytesWritten)
{
    if (!node.handler_) {
        return false;
    }

    auto& el = driver_.eventLoop();
    bool postResult =
        el.post(std::bind(std::move(node.handler_), status, bytesWritten));
    static_cast<void>(postResult);
    GASSERT(postResult);
    GASSERT(!node.handler_);
    return true;
}

}  // namespace io

}  // namespace embxx

