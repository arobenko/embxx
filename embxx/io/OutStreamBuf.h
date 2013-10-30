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


#pragma once

#include <cstddef>
#include <algorithm>

#include "embxx/container/StaticQueue.h"
#include "embxx/util/StaticFunction.h"

namespace embxx
{

namespace io
{

template <typename TDriver,
          std::size_t TBufSize,
          typename TWaitHandler = embxx::util::StaticFunction<void ()> >
class OutStreamBuf
{
public:
    typedef TDriver Driver;
    typedef typename Driver::CharType CharType;
    static const std::size_t BufSize = TBufSize;
    typedef TWaitHandler WaitHandler;

    typedef embxx::container::StaticQueue<CharType, BufSize> Buffer;

    typedef typename Buffer::Iterator Iterator;
    typedef Iterator iterator;
    typedef typename Buffer::ConstIterator ConstIterator;
    typedef ConstIterator const_iterator;
    typedef typename Buffer::ValueType ValueType;
    typedef ValueType value_type;
    typedef typename Buffer::Reference Reference;
    typedef Reference reference;
    typedef typename Buffer::ConstReference ConstReference;
    typedef ConstReference const_reference;


    OutStreamBuf(Driver& driver);
    ~OutStreamBuf();

    OutStreamBuf(const OutStreamBuf&) = default;

    OutStreamBuf& operator=(const OutStreamBuf&) = delete;

    Driver& driver();

    const Driver& driver() const;

    std::size_t size() const;

    std::size_t availableCapacity() const;

    constexpr std::size_t fullCapacity() const;

    std::size_t resize(std::size_t newSize);

    void flush(std::size_t flushSize);

    void flush();

    std::size_t pushBack(const CharType* str);

    std::size_t push_back(const CharType* str);

    std::size_t pushBack(const CharType* str, std::size_t strSize);

    std::size_t push_back(const CharType* str, std::size_t strSize);

    std::size_t pushBack(CharType ch);

    std::size_t push_back(CharType ch);

    Iterator begin();

    Iterator end();

    ConstIterator begin() const;

    ConstIterator end() const;

    ConstIterator cbegin() const;

    ConstIterator cend() const;

    Reference operator[](std::size_t idx);

    ConstReference operator[](std::size_t idx) const;

    template <typename TFunc>
    void asyncWaitAvailableCapacity(
        std::size_t capacity,
        TFunc&& func);

private:

    void initiateFlush();

    Driver& driver_;
    Buffer buf_;
    std::size_t flushedSize_;
    std::size_t waitAvailableCapacity_;
    WaitHandler waitHandler_;
};

// Implementation
template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::OutStreamBuf(Driver& driver)
    : driver_(driver),
      flushedSize_(0),
      waitAvailableCapacity_(0)
{
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::~OutStreamBuf()
{
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename OutStreamBuf<TDriver, TBufSize, TWaitHandler>::Driver&
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::driver()
{
    return driver_;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
const typename OutStreamBuf<TDriver, TBufSize, TWaitHandler>::Driver&
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::driver() const
{
    return driver_;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
std::size_t OutStreamBuf<TDriver, TBufSize, TWaitHandler>::size() const
{
    return buf_.size() - flushedSize_;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
std::size_t OutStreamBuf<TDriver, TBufSize, TWaitHandler>::availableCapacity() const
{
    return buf_.capacity() - flushedSize_;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
constexpr std::size_t
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::fullCapacity() const
{
    return buf_.capacity();
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
std::size_t OutStreamBuf<TDriver, TBufSize, TWaitHandler>::resize(
    std::size_t newSize)
{
    auto minSize = std::min(fullCapacity(), size + flushedSize_);
    buf_.resize(minSize);
    return size();
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
void OutStreamBuf<TDriver, TBufSize, TWaitHandler>::flush(
    std::size_t flushSize)
{
    bool flush = (flushedSize_ == 0);
    std::size_t actualFlushSize = std::min(flushSize, size());
    flushedSize_ += actualFlushSize;
    if (flush) {
        initiateFlush();
    }
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
void OutStreamBuf<TDriver, TBufSize, TWaitHandler>::flush()
{
    flush(size());
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
std::size_t OutStreamBuf<TDriver, TBufSize, TWaitHandler>::pushBack(
    const CharType* str)
{
    std::size_t count = 0;
    while ((*str != static_cast<CharType>(0) && (!buf_.isFull()))) {
        pushBack(*str);
        ++str;
        ++count;
    }
    return count;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
std::size_t OutStreamBuf<TDriver, TBufSize, TWaitHandler>::push_back(
    const CharType* str)
{
    return pushBack(str);
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
std::size_t OutStreamBuf<TDriver, TBufSize, TWaitHandler>::pushBack(
    const CharType* str, std::size_t strSize)
{
    auto sizeToWrite = std::min(strSize, buf_.capacity() - buf_.size());
    for (auto count = 0U; count < sizeToWrite; ++count) {
        buf_.pushBack(*str);
        ++str;
    }
    return sizeToWrite;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
std::size_t OutStreamBuf<TDriver, TBufSize, TWaitHandler>::push_back(
    const CharType* str, std::size_t strSize)
{
    return pushBack(str, strSize);
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
std::size_t OutStreamBuf<TDriver, TBufSize, TWaitHandler>::pushBack(
    CharType ch)
{
    if (buf_.isFull()) {
        return 0;
    }
    buf_.pushBack(ch);
    return 1;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
std::size_t OutStreamBuf<TDriver, TBufSize, TWaitHandler>::push_back(
    CharType ch)
{
    return pushBack(ch);
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename OutStreamBuf<TDriver, TBufSize, TWaitHandler>::Iterator
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::begin()
{
    return buf_.begin() + flushedSize_;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename OutStreamBuf<TDriver, TBufSize, TWaitHandler>::Iterator
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::end()
{
    return buf_.end();
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename OutStreamBuf<TDriver, TBufSize, TWaitHandler>::ConstIterator
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::begin() const
{
    return cbegin();
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename OutStreamBuf<TDriver, TBufSize, TWaitHandler>::ConstIterator
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::end() const
{
    return cend();
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename OutStreamBuf<TDriver, TBufSize, TWaitHandler>::ConstIterator
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::cbegin() const
{
    return buf_.cbegin() + flushedSize_;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename OutStreamBuf<TDriver, TBufSize, TWaitHandler>::ConstIterator
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::cend() const
{
    return buf_.cend();
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename OutStreamBuf<TDriver, TBufSize, TWaitHandler>::Reference
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::operator[](std::size_t idx)
{
    return buf_[idx + flushedSize_];
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename OutStreamBuf<TDriver, TBufSize, TWaitHandler>::ConstReference
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::operator[](
    std::size_t idx) const
{
    return buf_[idx + flushedSize_];
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
template <typename TFunc>
void OutStreamBuf<TDriver, TBufSize, TWaitHandler>::asyncWaitAvailableCapacity(
    std::size_t capacity,
    TFunc&& func)
{
    GASSERT(capacity <= fullCapacity());
    GASSERT(!waitHandler_);
    if (capacity <= availableCapacity()) {
        auto postResult = driver_.eventLoop().post(std::forward<TFunc>(func));
        GASSERT((postResult) || (!"Failed to post handler, increase size of Event Loop"));
        static_cast<void>(postResult);
    }
    waitAvailableCapacity_ = capacity;
    waitHandler_ = std::forward<TFunc>(func);
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
void OutStreamBuf<TDriver, TBufSize, TWaitHandler>::initiateFlush()
{
    static const std::size_t MaxSingleFlushSize = BufSize / 4;
    auto rangeOne = buf_.arrayOne();
    auto distance = static_cast<std::size_t>(
        std::distance(rangeOne.first, rangeOne.second));
    auto flushSize = std::min(std::size_t(MaxSingleFlushSize), distance);
    flushSize = std::min(flushSize, flushedSize_);

    driver_.asyncWrite(&buf_.front(), flushSize,
        [this](embxx::driver::ErrorStatus status, std::size_t bytesWritten)
        {
            static_cast<void>(status); // Status doesn't matter;
            GASSERT(bytesWritten <= flushedSize_);
            GASSERT(bytesWritten <= buf_.size());
            flushedSize_ -= bytesWritten;
            buf_.popFront(bytesWritten);
            if ((waitHandler_) &&
                (waitAvailableCapacity_ <= availableCapacity())) {
                auto& el = driver_.eventLoop();
                auto postResult = el.post(std::move(waitHandler_));
                GASSERT((postResult) || (!"Failed to post handler, increase size of Event Loop"));
                static_cast<void>(postResult);

                GASSERT(!waitHandler_);
            }

            if (0 < flushedSize_) {
                initiateFlush();
            }
        });
}

}  // namespace io

}  // namespace embxx


