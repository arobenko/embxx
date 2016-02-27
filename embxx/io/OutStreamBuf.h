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
#include "embxx/error/ErrorStatus.h"

namespace embxx
{

namespace io
{

/// @addtogroup io
/// @{

/// @brief Output Stream Buffer
/// @details This class implements functionality of output stream buffer
///          without use of dynamic memory allocation, RTTI and/or exceptions.
/// @tparam TDriver Class of the driver (such as embxx::driver::Character), it
///         must provide the following API functions and types:
///         @code
///         // Character type
///         typedef ... CharType;
///
///         // Asynchronous write request
///         template <typename TFunc>
///         void asyncWrite(const CharType* buf, std::size_t size, TFunc&& func);
///
///         // Get reference to event loop object
///         EventLoop& eventLoop();
///         @endcode
/// @tparam TBufSize Size of the internal buffer in number of characters it
///         may contain.
/// @tparam TWaitHandler Callback functor class to be called when requested
///         space becomes available. Must be either
///         std::function or embxx::util::StaticFunction and have
///         "void (const embxx::error::ErrorStatus&)" signature. It is used to store
///         callback handler provided in asyncWaitAvailableCapacity() request.
/// @pre No other components performs asynchronous write requests to the same
///      driver.
/// @headerfile embxx/io/OutStreamBuf.h
template <typename TDriver,
          std::size_t TBufSize,
          typename TWaitHandler = embxx::util::StaticFunction<void (const embxx::error::ErrorStatus&)> >
class OutStreamBuf
{
public:
    /// @brief Type of the driver
    typedef TDriver Driver;

    /// @brief Type of single character
    typedef typename Driver::CharType CharType;

    /// @brief Size of the internal buffer
    static const std::size_t BufSize = TBufSize;

    /// @brief Type of the wait handler
    typedef TWaitHandler WaitHandler;

    /// @brief Type of the internal circular buffer
    typedef embxx::container::StaticQueue<CharType, BufSize> Buffer;

    /// @brief Type of the iterator
    typedef typename Buffer::Iterator Iterator;

    /// @brief Same as Iterator
    typedef Iterator iterator;

    /// @brief Type of const Iterator
    typedef typename Buffer::ConstIterator ConstIterator;

    /// @brief Same as ConstIterator
    typedef ConstIterator const_iterator;

    /// @brief Type of values (characters) storred in internal buffer
    typedef typename Buffer::ValueType ValueType;

    /// @brief Same as ValueType
    typedef ValueType value_type;

    /// @brief Reference type
    typedef typename Buffer::Reference Reference;

    /// @brief Same as Reference
    typedef Reference reference;

    /// @brief Const reference type
    typedef typename Buffer::ConstReference ConstReference;

    /// @brief Same as ConstReference
    typedef ConstReference const_reference;

    /// @brief Constructor
    /// @param driv Reference to driver object
    explicit OutStreamBuf(Driver& driv);

    /// @brief Destructor
    ~OutStreamBuf();

    /// @brief Copy constructor is default
    OutStreamBuf(const OutStreamBuf&) = default;

    /// @brief Move constructor is default
    OutStreamBuf(OutStreamBuf&&) = default;

    /// @brief Copy assignment operator is deleted
    OutStreamBuf& operator=(const OutStreamBuf&) = delete;

    /// @brief Get reference to the driver object.
    Driver& driver();

    /// @brief Const version of driver()
    const Driver& driver() const;

    /// @brief Get size of modifiable section in the internal buffer
    std::size_t size() const;

    /// @brief Check whether size of the modifiable section is 0.
    /// @return true if and only if size() == 0.
    bool empty() const;

    /// @brief Clear modifiable section of the internal buffer
    /// @post Call to size() returns 0;
    void clear();

    /// @brief Get current available space in the buffer until it becomes full.
    std::size_t availableCapacity() const;

    /// @brief Get full capacity of the buffer
    /// @details Equals to TSizeBuf template parameter of the class
    constexpr std::size_t fullCapacity() const;

    /// @brief Resize available for modification part of the buffer
    std::size_t resize(std::size_t newSize);

    /// @brief Flush the contents of the buffer to be written to the device.
    /// @param flushSize Number of characters from the "modifiable" part of the
    ///        buffer to be flushed to the device.
    /// @pre flushSize must be less or equal to the value returned by size()
    ///      prior to this function call.
    /// @post The "flushed" part of the buffer becomes unaccessible via
    ///       operator[], begin(), end() functions.
    /// @post The "flushed" part of the buffer mustn't be modified by any
    ///       external means until the write is complete.
    void flush(std::size_t flushSize);

    /// @brief Flush the whole buffer.
    /// @details Equivalent to flash(size()).
    void flush();

    /// @brief Push back "C" (0-terminated) string
    /// @details The final 0 is not pushed.
    /// @param str 0 terminated string
    /// @return Number of characters written, may be less than length of the
    ///         string (in case the internal buffer becomes full).
    std::size_t pushBack(const CharType* str);

    /// @copydoc pushBack(const CharType*)
    std::size_t push_back(const CharType* str);

    /// @brief Push back buffer of characters.
    /// @param str Pointer to buffer of characters
    /// @param strSize Size of the buffer
    /// @return Number of characters written, may be less than strSize parameter
    ///         (in case the internal buffer becomes full).
    std::size_t pushBack(const CharType* str, std::size_t strSize);

    /// @copydoc pushBack(const CharType*, std::size_t)
    std::size_t push_back(const CharType* str, std::size_t strSize);

    /// @brief Push back single character.
    /// @param ch Character
    /// @return 1 in case of success, 0 in case internal buffer is full
    std::size_t pushBack(CharType ch);

    /// @copydoc pushBack(CharType)
    std::size_t push_back(CharType ch);

    /// @brief Returns iterator to the beginning of "modifiable" (not flushed)
    ///        section in the buffer.
    Iterator begin();

    /// @brief Returns iterator to the end of "modifiable" (not flushed)
    ///        section of the buffer
    Iterator end();

    /// @brief Same as cbegin()
    ConstIterator begin() const;

    /// @brief Same as cend()
    ConstIterator end() const;

    /// @brief Const version of begin()
    ConstIterator cbegin() const;

    /// @brief Const version of end()
    ConstIterator cend() const;

    /// @brief Operator to access element in the "modifiable" (not flushed)
    ///        section of the buffer.
    /// @param idx Index of the element.
    /// @pre @code idx < size() @endcode
    Reference operator[](std::size_t idx);

    /// @brief Const version of operator[]
    ConstReference operator[](std::size_t idx) const;

    /// @brief Asynchronous wait until requested capacity of the internal
    ///        buffer becomes available.
    /// @details The function records copies the callback object to its internal
    ///          data structures and returns immediately. The callback will
    ///          be called in the context of the event loop when requested
    ///          size becomes available.
    /// @param capacity Requested capacity.
    /// @param func Callback functor object, must have
    ///        "void (const embxx::error::ErrorStatus&)" signature.
    /// @pre All the previous asynchronous wait request are complete (their
    ///      callback has been executed).
    /// @pre @code capacity <= fullCapacity() @endcode
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

/// @}

// Implementation
template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::OutStreamBuf(Driver& driv)
    : driver_(driv),
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
bool OutStreamBuf<TDriver, TBufSize, TWaitHandler>::empty() const
{
    return (size() == 0U);
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
void OutStreamBuf<TDriver, TBufSize, TWaitHandler>::clear()
{
    buf_.popBack(size());
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
    auto minSize = std::min(fullCapacity(), newSize + flushedSize_);
    buf_.resize(minSize);
    return size();
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
void OutStreamBuf<TDriver, TBufSize, TWaitHandler>::flush(
    std::size_t flushSize)
{
    GASSERT(flushSize <= size());
    bool flushRequired = (flushedSize_ == 0);
    std::size_t actualFlushSize = std::min(flushSize, size());
    flushedSize_ += actualFlushSize;
    if (flushRequired) {
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
    GASSERT(idx < size());
    return buf_[idx + flushedSize_];
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename OutStreamBuf<TDriver, TBufSize, TWaitHandler>::ConstReference
OutStreamBuf<TDriver, TBufSize, TWaitHandler>::operator[](
    std::size_t idx) const
{
    GASSERT(idx < size());
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
        auto postResult =
            driver_.eventLoop().post(
                std::bind(std::forward<TFunc>(func), embxx::error::ErrorCode::Success));
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
        [this](const embxx::error::ErrorStatus& status, std::size_t bytesWritten)
        {
            GASSERT(bytesWritten <= flushedSize_);
            GASSERT(bytesWritten <= buf_.size());
            flushedSize_ -= bytesWritten;
            buf_.popFront(bytesWritten);
            if ((waitHandler_) &&
                (waitAvailableCapacity_ <= availableCapacity())) {
                auto& el = driver_.eventLoop();
                auto postResult =
                    el.post(
                        std::bind(
                            std::move(waitHandler_),
                            status));
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


