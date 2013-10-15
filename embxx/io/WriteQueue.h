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

#include "embxx/driver/ErrorStatus.h"
#include "embxx/util/StaticFunction.h"
#include "embxx/container/StaticQueue.h"

namespace embxx
{

namespace io
{

template <typename TDriver,
          std::size_t TSize,
          typename THandler = util::StaticFunction<void (embxx::driver::ErrorStatus, std::size_t)> >
class WriteQueue
{
public:
    typedef TDriver Driver;
    static const std::size_t Size = TSize;
    typedef THandler Handler;

    typedef typename Driver::CharType CharType;
    typedef unsigned WriteHandle;

    static const WriteHandle InvalidWriteHandle =
        std::numeric_limits<WriteHandle>::max();

    WriteQueue(Driver& driver);
    ~WriteQueue();

    template <typename TFunc>
    WriteHandle asyncWrite(
        const CharType* buf,
        std::size_t size,
        TFunc&& func);

    WriteHandle asyncWrite(
        const CharType* buf,
        std::size_t size);

    bool cancel(WriteHandle handle);

    void cancelAll();

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
        embxx::driver::ErrorStatus status,
        std::size_t bytesWritten);


    Driver& driver_;
    Queue queue_;
    WriteHandle nextHandleId_;
};

// Implementation
template <typename TDriver,
          std::size_t TSize,
          typename THandler>
WriteQueue<TDriver, TSize, THandler>::WriteQueue(Driver& driver)
    : driver_(driver),
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
        [](embxx::driver::ErrorStatus es, std::size_t bytesWritten)
        {
            static_cast<void>(es);
            static_cast<void>(bytesWritten);
        });
}

template <typename TDriver,
          std::size_t TSize,
          typename THandler>
bool WriteQueue<TDriver, TSize, THandler>::cancel(WriteHandle handle)
{
    if (queue_.isEmpty()) {
        return false;
    }

    auto findNodeFunc = [handle](typename Queue::IteratorRange range) -> typename Queue::Iterator
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
        return invokeHandler(*iter, embxx::driver::ErrorStatus::Aborted, 0);
    }

    auto arrayTwo = queue_.arrayTwo();
    iter = findNodeFunc(arrayTwo);
    if (iter == arrayTwo.second) {
        return false;
    }

    return invokeHandler(*iter, embxx::driver::ErrorStatus::Aborted, 0);
}

template <typename TDriver,
          std::size_t TSize,
          typename THandler>
void WriteQueue<TDriver, TSize, THandler>::cancelAll()
{
    if (queue_.isEmpty()) {
        return false;
    }

    return driver_.cancelWrite();

    auto cancelAllFunc =
        [this](typename Queue::Iterator iterBegin, typename Queue::Iterator iterEnd)
        {
            std::for_each(iterBegin, iterEnd,
                [this](Node& node)
                {
                    auto result =
                        invokeHandler(node, embxx::driver::ErrorStatus::Aborted, 0);
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
            [this](embxx::driver::ErrorStatus es, std::size_t bytesWritten)
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
    auto func = [this](typename Queue::IteratorRange range) -> bool
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
    embxx::driver::ErrorStatus status,
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

