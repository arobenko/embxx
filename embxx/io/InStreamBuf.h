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
#include <limits>

#include "embxx/container/StaticQueue.h"
#include "embxx/util/StaticFunction.h"
#include "embxx/error/ErrorStatus.h"

namespace embxx
{

namespace io
{

/// @ingroup io
/// @brief Input Stream Buffer
/// @details This class implements functionality of input stream buffer
///          without use of dynamic memory allocation, RTTI and/or exceptions.
/// @tparam TDriver Class of the driver (such as embxx::driver::Character), it
///         must provide the following API functions and types:
///         @code
///         // Character type
///         typedef ... CharType;
///
///         // Asynchronous write request
///         template <typename TFunc>
///         void asyncRead(CharType* buf, std::size_t size, TFunc&& func);
///
///         // Get reference to event loop object
///         EventLoop& eventLoop();
///         @endcode
/// @tparam TBufSize Size of the internal buffer in number of characters it
///         may contain.
/// @tparam TWaitHandler Callback functor class to be called when requested
///         data amount becomes available. Must be either
///         std::function or embxx::util::StaticFunction and have
///         "void (const embxx::error::ErrorStatus&)" signature. It is used to
///         store callback handler provided in asyncWaitDataAvailable() request.
/// @pre No other components performs asynchronous read requests to the same
///      driver.
/// @headerfile embxx/io/InStreamBuf.h
template <typename TDriver,
          std::size_t TBufSize,
          typename TWaitHandler = embxx::util::StaticFunction<void (const embxx::error::ErrorStatus&)> >
class InStreamBuf
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

    /// @brief Type of const Iterator
    typedef typename Buffer::ConstIterator ConstIterator;

    /// @brief Same as ConstIterator
    typedef ConstIterator const_iterator;

    /// @brief Type of values (characters) storred in internal buffer
    typedef typename Buffer::ValueType ValueType;

    /// @brief Same as ValueType
    typedef ValueType value_type;

    /// @brief Const reference type
    typedef typename Buffer::ConstReference ConstReference;

    /// @brief Same as ConstReference
    typedef ConstReference const_reference;

    /// @brief Constructor
    /// @param driv Reference to driver object
    explicit InStreamBuf(Driver& driv);

        /// @brief Destructor
    ~InStreamBuf();

    /// @brief Copy constructor is default
    InStreamBuf(const InStreamBuf&) = default;

    /// @brief Move constructor is default
    InStreamBuf(InStreamBuf&&) = default;

    /// @brief Copy assignment operator is deleted
    InStreamBuf& operator=(const InStreamBuf&) = delete;

    /// @brief Get reference to the driver object.
    Driver& driver();

    /// @brief Const version of driver()
    const Driver& driver() const;

    /// @brief Get size of available for read data.
    std::size_t size() const;

    /// @brief Check whether number of available characters is 0.
    /// @return true if and only if size() == 0.
    bool empty() const;

    /// @brief Get full capacity of the buffer
    /// @details Equals to TSizeBuf template parameter of the class
    constexpr std::size_t fullCapacity() const;

    /// @brief Consume part of the available data for read.
    /// @param consumeSize Number of characters from the from the available
    ///        data to be removed from the buffer.
    /// @pre consumeSize must be less or equal to the value returned by size()
    ///      prior to this function call.
    /// @post The "consumed" part of the buffer becomes unaccessible via
    ///       operator[], begin(), end() functions.
    void consume(std::size_t consumeSize);

    /// @brief Consume the whole buffer.
    /// @details Equivalent to consume(size()).
    void consume();

    /// @brief Start data accumulation in the internal buffer.
    /// @details This function activates read from the actual device to allow
    ///          data accumulation until it is needed. If the internal buffer
    ///          becomes full all subsequent incoming data is ignored.
    /// @pre The data accumulated hasn't been already started before:
    ///      @code isRunning() == false @endcode
    void start();

    /// @brief Stop data accumulation in the internal buffer.
    /// @details This function stops using actual device to accumulate incomming
    ///          data in the internal buffer.
    /// @pre Data accumulation has been previously started by start():
    ///      @code isRunning() == true @endcode
    void stop();

    /// @brief Check whether current stream buffer is in a process of accumulating
    ///        incoming data in its internal data buffer.
    bool isRunning() const;

    /// @brief Perform asynchronous wait until requested data size becomes
    ///        available for processing.
    /// @details The function records copies the callback object to its internal
    ///          data structures and returns immediately. The callback will
    ///          be called in the context of the event loop when requested
    ///          data size becomes available.
    /// @param reqSize Requested available data size
    /// @param func Callback functor object,
    ///        must have "void (const embxx::error::ErrorStatus&)" signature.
    /// @pre The data accumulation has been started earlier using start()
    ///      member function.
    /// @pre All the previous asynchronous wait request are complete (their
    ///      callback has been executed).
    /// @pre @code reqSize <= fullCapacity() @endcode
    template <typename TFunc>
    void asyncWaitDataAvailable(
        std::size_t reqSize,
        TFunc&& func);

    /// @brief Same as cbegin()
    ConstIterator begin() const;

    /// @brief Same as cend();
    ConstIterator end() const;

    /// @brief Returns const iterator to the beginning of "readable"
    ///        (not yet consumed) section in the buffer.
    ConstIterator cbegin() const;

    /// @brief Returns const iterator to the end of "readable"
    ///        (not yet consumed) section in the buffer.
    ConstIterator cend() const;

    /// @brief Operator to access element in the "readable" (not yet consumed)
    ///        section of the buffer.
    /// @param idx Index of the element.
    /// @pre @code idx < size() @endcode
    ConstReference operator[](std::size_t idx) const;

private:
    void startAsyncRead();
    void invokeHandler(const embxx::error::ErrorStatus& status);

    Driver& driver_;
    Buffer buf_;
    std::size_t availableSize_;
    WaitHandler waitHandler_;
    std::size_t waitAvailableDataSize_;
    bool running_;
    bool readInProgress_;
};

// Implementation
template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
InStreamBuf<TDriver, TBufSize, TWaitHandler>::InStreamBuf(Driver& driv)
    : driver_(driv),
      availableSize_(0),
      waitAvailableDataSize_(std::numeric_limits<decltype(waitAvailableDataSize_)>::max()),
      running_(false),
      readInProgress_(false)
{
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
InStreamBuf<TDriver, TBufSize, TWaitHandler>::~InStreamBuf()
{
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename InStreamBuf<TDriver, TBufSize, TWaitHandler>::Driver&
InStreamBuf<TDriver, TBufSize, TWaitHandler>::driver()
{
    return driver_;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
const typename InStreamBuf<TDriver, TBufSize, TWaitHandler>::Driver&
InStreamBuf<TDriver, TBufSize, TWaitHandler>::driver() const
{
    return driver_;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
std::size_t InStreamBuf<TDriver, TBufSize, TWaitHandler>::size() const
{
    GASSERT(availableSize_ <= buf_.size());
    return availableSize_;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
bool InStreamBuf<TDriver, TBufSize, TWaitHandler>::empty() const
{
    return (size() == 0U);
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
constexpr std::size_t
InStreamBuf<TDriver, TBufSize, TWaitHandler>::fullCapacity() const
{
    return buf_.capacity();
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
void InStreamBuf<TDriver, TBufSize, TWaitHandler>::consume(
    std::size_t consumeSize)
{
    GASSERT(consumeSize <= size());
    auto actualConsumeSize = std::min(consumeSize, size());
    availableSize_ -= actualConsumeSize;
    buf_.popFront(actualConsumeSize);
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
void InStreamBuf<TDriver, TBufSize, TWaitHandler>::consume()
{
    consume(size());
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
void InStreamBuf<TDriver, TBufSize, TWaitHandler>::start()
{
    GASSERT(!isRunning());
    running_ = true;
    if (!readInProgress_) {
        startAsyncRead();
    }
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
void InStreamBuf<TDriver, TBufSize, TWaitHandler>::stop()
{
    GASSERT(isRunning());
    driver_.cancelRead();
    invokeHandler(embxx::error::ErrorCode::Aborted);
    running_ = false;;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
bool InStreamBuf<TDriver, TBufSize, TWaitHandler>::isRunning() const
{
    return running_;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
template <typename TFunc>
void InStreamBuf<TDriver, TBufSize, TWaitHandler>::asyncWaitDataAvailable(
    std::size_t reqSize,
    TFunc&& func)
{
    GASSERT(isRunning());
    GASSERT(!waitHandler_);
    GASSERT(reqSize <= fullCapacity());
    waitHandler_ = std::forward<TFunc>(func);
    if (reqSize <= availableSize_) {
        invokeHandler(embxx::error::ErrorCode::Success);
        return;
    }

    waitAvailableDataSize_ = reqSize;
    driver_.cancelRead(); // The next wait will be reprogrammed in handler.
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename InStreamBuf<TDriver, TBufSize, TWaitHandler>::ConstIterator
InStreamBuf<TDriver, TBufSize, TWaitHandler>::begin() const
{
    return cbegin();
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename InStreamBuf<TDriver, TBufSize, TWaitHandler>::ConstIterator
InStreamBuf<TDriver, TBufSize, TWaitHandler>::end() const
{
    return cend();
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename InStreamBuf<TDriver, TBufSize, TWaitHandler>::ConstIterator
InStreamBuf<TDriver, TBufSize, TWaitHandler>::cbegin() const
{
    return buf_.cbegin();
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename InStreamBuf<TDriver, TBufSize, TWaitHandler>::ConstIterator
InStreamBuf<TDriver, TBufSize, TWaitHandler>::cend() const
{
    GASSERT(availableSize_ <= buf_.size());
    return cbegin() + availableSize_;
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
typename InStreamBuf<TDriver, TBufSize, TWaitHandler>::ConstReference
InStreamBuf<TDriver, TBufSize, TWaitHandler>::operator[](
    std::size_t idx) const
{
    GASSERT(idx < size());
    return buf_[idx];
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
void InStreamBuf<TDriver, TBufSize, TWaitHandler>::startAsyncRead()
{
    GASSERT(!readInProgress_);
    static const std::size_t DefaultReadSize =
        std::max(buf_.capacity() / 4, std::size_t(1U));
    std::size_t nextReadSize =
        std::min(DefaultReadSize, buf_.capacity() - availableSize_);

    buf_.resize(availableSize_ + nextReadSize);
    auto rangeOne = buf_.arrayOne();
    auto rangeOneDistance =
        static_cast<std::size_t>(std::distance(rangeOne.first, rangeOne.second));

    CharType* readPtr = nullptr;
    if (availableSize_ < rangeOneDistance) {
        readPtr = &(*(rangeOne.first + availableSize_));
        nextReadSize = std::min(nextReadSize, rangeOneDistance - availableSize_);
    }
    else {
        auto rangeTwo = buf_.arrayTwo();
        auto rangeTwoDistance =
            static_cast<std::size_t>(std::distance(rangeTwo.first, rangeTwo.second));

        auto rangeTwoSize = availableSize_ - rangeOneDistance;
        GASSERT(rangeTwoSize <= rangeTwoDistance);
        readPtr = &(*(rangeTwo.first + rangeTwoSize));
        nextReadSize = std::min(nextReadSize, rangeTwoDistance - rangeTwoSize);
    }

    if (waitHandler_) {
        GASSERT(availableSize_ < waitAvailableDataSize_);
        nextReadSize = std::min(nextReadSize, waitAvailableDataSize_ - availableSize_);
    }

    GASSERT(readPtr != nullptr);

    readInProgress_ = true;
    driver_.asyncRead(readPtr, nextReadSize,
        [this](const embxx::error::ErrorStatus& es, std::size_t bytesRead)
        {
            readInProgress_ = false;

            GASSERT(availableSize_ + bytesRead <= buf_.capacity());
            availableSize_ += bytesRead;

            if (waitHandler_) {
                if (waitAvailableDataSize_ <= availableSize_) {
                    invokeHandler(embxx::error::ErrorCode::Success);
                }
                else if (es && (es.code() != embxx::error::ErrorCode::Aborted)) {
                    invokeHandler(es);
                }
                else {
                    // The remaining cases are not enough data and/or read operation
                    // aborted should not be reported back.
                }
            }

            if (!isRunning()) {
                GASSERT(!waitHandler_);
                return;
            }

            startAsyncRead();
        });
}

template <typename TDriver, std::size_t TBufSize, typename TWaitHandler>
void InStreamBuf<TDriver, TBufSize, TWaitHandler>::invokeHandler(
    const embxx::error::ErrorStatus& status)
{
    if (waitHandler_) {
        auto postResult = driver_.eventLoop().post(
            std::bind(std::move(waitHandler_), status));
        static_cast<void>(postResult);
        GASSERT(postResult);
        GASSERT(!waitHandler_);
        waitAvailableDataSize_ =
            std::numeric_limits<decltype(waitAvailableDataSize_)>::max();
    }
}


}  // namespace io

}  // namespace embxx


