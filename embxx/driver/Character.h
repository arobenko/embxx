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

/// @file embxx/driver/Character.h
/// Put file contains definition of "Character" device driver class.

#pragma once

#include <functional>
#include "embxx/util/StaticFunction.h"
#include "embxx/util/Assert.h"
#include "embxx/error/ErrorStatus.h"

namespace embxx
{

namespace driver
{

/// @addtogroup driver
/// @{

/// @brief Character device driver
/// @details Manages read/write operations on the character device (peripheral)
///          such as RS-232.
/// @tparam TDevice Platform specific device (peripheral) control class. It
///         must expose the following interface:
///         @code
///         // Define each character type as CharType
///         typedef char CharType;
///
///         // Start read operation. The device will perform some configuration
///         // if needed and enable read interrupts, i.e. interrupt when there
///         // is at least one character to read. This function is called in
///         // NON-interrupt context.
///         void startRead(std::size_t length);
///
///         // Cancel read operation. The return value indicates whether the
///         // read operation was cancelled. This function is called in
///         // NON-interrupt context.
///         bool cancelRead();
///
///         // Same as cancelRead(), but called in interrupt context from within
///         // the "can read" callback when asyncReadUntil() function is used
///         // to perform read and requested character is recognised.
///         bool cancelReadInterruptCtx();
///
///         // Start write operation. The device will perform some configuration
///         // if needed and enable write interrupts, i.e. interrupt when there
///         // is space for at least one character to be written. This function
///         // is called in NON-interrupt context.
///         void startWrite(std::size_t length);
///
///         // Cancel write operation. The return value indicates whether the
///         // write operation was cancelled. This function is called in
///         // NON-interrupt context.
///         bool cancelWrite();
///
///         // Set the "can read" interrupt callback. Expose signature
///         // "void ()". The callback must be
///         // called when there is at least one byte available for read
///         // or hardware protocol error reported. The callback will perform
///         //  multiple canRead() and read() calls until canRead() returns
///         // false. This function is called in NON-interrupt context, but the
///         // callback itself will be called by the device control object in
///         // interrupt context.
///         template <typename TFunc>
///         void setCanReadHandler(TFunc&& func);
///
///         // Set the "can write" interrupt callback. Expose signature
///         // "void ()".The callback must be
///         // called when there is a space for at least one byte to be written
///         // or hardware protocol error reported. The callback will perform
///         // multiple canWrite() and write() calls until canWrite() returns
///         // false. This function is called in NON-interrupt context, but the
///         // callback itself will be called by the device control object in
///         // interrupt context.
///         template <typename TFunc>
///         void setCanWriteHandler(TFunc&& func);
///
///         // Set the "read complete" interrupt callback. Expose signature
///         // "void (const embxx::error::ErrorStatus&)". The callback must be
///         // called when read operation is complete and read interrupts are
///         // disabled, i.e. no more "can read" callback calls will follow
///         // until next call to startRead(). This function is called in
///         // NON-interrupt context, but the callback itself will be called
///         // by the device control object in interrupt context.
///         template <typename TFunc>
///         void setReadCompleteHandler(TFunc&& func);
///
///         // Set the "write complete" interrupt callback. Expose signature
///         // "void (const embxx::error::ErrorStatus&)". The callback must be
///         // called when write operation is complete and write interrupts are
///         // disabled, i.e. no more "can write" callback calls will follow
///         // until next call to startWrite(). This function is called in
///         // NON-interrupt context, but the callback itself will be called
///         // by the device control object in interrupt context.
///         template <typename TFunc>
///         void setWriteCompleteHandler(TFunc&& func);
///
///         // Inquiry whether there is at least one character to be
///         // read. Will be called in the interrupt context. May be called
///         // multiple times in the same interrupt.
///         bool canRead();
///
///         // Inquiry whether there is a space for at least one character to
///         // be written. Will be called in the interrupt context. May be called
///         // multiple times in the same interrupt.
///         bool canWrite();
///
///         // Read one character. Precondition to this call: canRead() returns
///         // true. Will be called in the interrupt context. May be called
///         // multiple times in the same interrupt.
///         CharType read();
///
///         // Write one character. Precondition to this call: canWrite() returns
///         // true. Will be called in the interrupt context. May be called
///         // multiple times in the same interrupt.
///         void write(CharType value);
///         @endcode
///
/// @tparam TEventLoop A variant of embxx::util::EventLoop object that is used
///         to execute posted handlers in regular thread context.
/// @tparam TReadHandler A function class that is supposed to store "read"
///         complete callback. Must be either std::function of embxx::util::StaticFunction
///         and provide "void(const embxx::error::ErrorStatus&, std::size_t)"
///         calling interface, where the first parameter is error status of the
///         operation and second one is how many bytes were actually read in
///         the operation.
/// @tparam TWriteHandler A function class that is supposed to store "write"
///         complete callback. Must be either std::function of embxx::util::StaticFunction
///         and provide "void(const embxx::error::ErrorStatus&, std::size_t)"
///         calling interface, where the first parameter is error status of the
///         operation and second one is how many bytes were actually written in
///         the operation.
/// @headerfile embxx/driver/Character.h
template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler = embxx::util::StaticFunction<void(const embxx::error::ErrorStatus&, std::size_t)>,
          typename TWriteHandler = embxx::util::StaticFunction<void(const embxx::error::ErrorStatus&, std::size_t)> >
class Character
{
public:

    /// @brief Device (peripheral) control class
    typedef TDevice Device;

    /// @brief Event loop class
    typedef TEventLoop EventLoop;

    /// @brief Type of read handler holder class
    typedef TReadHandler ReadHandler;

    /// @brief Type of write handler holder class
    typedef TWriteHandler WriteHandler;

    /// @brief type of single character
    typedef typename Device::CharType CharType;

    /// @brief Constructor
    /// @param device Reference to device (peripheral) control object
    /// @param el Reference to event loop object
    Character(Device& device, EventLoop& el);

    /// @brief Copy constructor is deleted.
    Character(const Character&) = delete;

    /// @brief MoveConstrutor is deleted
    Character(Character&&) = delete;

    /// @brief Copy assignment is deleted
    Character& operator=(const Character&) = delete;

    /// @brief Move assignment is deleted
    Character& operator=(Character&&) = delete;

    /// @brief Get reference to device (peripheral) control object.
    Device& device();

    /// @brief Get referent to event loop object.
    EventLoop& eventLoop();

    /// @brief Asynchronous read request
    /// @details The function returns immediately. The callback will be called
    ///          with operation results only when the provided buffer is full
    ///          or operation has been cancelled.
    /// @param buf Pointer to the output buffer. The buffer mustn't be
    ///            used or destructed until the callback has been called.
    /// @param size Size of the buffer.
    /// @param func Callback functor that must have following signature:
    ///        @code void callback(const embxx::error::ErrorStatus& status, std::size_t bytesRead); @endcode
    /// @pre The callback for any previous asyncRead() request has been called,
    ///      i.e. there is no outstanding asyncRead() request.
    /// @pre The provided buffer must stay valid and unused until the provided
    ///      callback function is called.
    template <typename TFunc>
    void asyncRead(
        CharType* buf,
        std::size_t size,
        TFunc&& func);

    /// @brief Asynchronous read until specific character request
    /// @details The function returns immediately. The callback will be called
    ///          with operation results if one of the three following conditions
    ///          is true:
    ///          @li the requested character was read.
    ///          @li the buffer is full
    ///          @li the read operation was cancelled by cancelRead().
    /// @param buf Pointer to the output buffer. The buffer mustn't be
    ///            used or destructed until the callback has been called.
    /// @param size Size of the buffer.
    /// @param untilChar Character that will terminate read request.
    /// @param func Callback functor that must have following signature:
    ///        @code void callback(const embxx::error::ErrorStatus& status, std::size_t bytesRead);@endcode
    /// @pre The callback for any previous asyncRead() request has been called,
    ///      i.e. there is no outstanding asyncRead() request.
    /// @pre The provided buffer must stay valid and unused until the provided
    ///      callback function is called.
    template <typename TFunc>
    void asyncReadUntil(
        CharType* buf,
        std::size_t size,
        CharType untilChar,
        TFunc&& func);

    /// @brief Cancel previous asynchronous read request (asyncRead())
    /// @details If there is no unfinished asyncRead() operation in progress
    ///          the call to this function will have no effect. Otherwise the
    ///          callback will be called with embxx::error::ErrorCode::Aborted
    ///          as status value.
    /// @return true in case the previous asyncRead() operation was really
    ///         cancelled, false in case there was no unfinished asynchronous
    ///         read request.
    bool cancelRead();

    /// @brief Asynchronous write request
    /// @details The function returns immediately. The callback will be called
    ///          with operation results only when all the data from provided
    ///          buffer has been sent or operation has been cancelled.
    /// @param buf Pointer to the read-only buffer. The buffer mustn't be
    ///            changed or destructed until the callback has been called.
    /// @param size Size of the buffer.
    /// @param func Callback functor that must have following signature:
    ///        @code void callback(const embxx::error::ErrorStatus& status, std::size_t bytesWritten); @endcode
    /// @pre The callback for any previous asyncWrite() request has been called,
    ///      i.e. there is no outstanding asyncWrite() request.
    /// @pre The provided buffer must stay valid and unused until the provided
    ///      callback function is called.
    template <typename TFunc>
    void asyncWrite(
        const CharType* buf,
        std::size_t size,
        TFunc&& func);

    /// @brief Cancel previous asynchronous write request (asyncWrite())
    /// @details If there is no unfinished asyncWrite() operation in progress
    ///          the call to this function will have no effect. Otherwise the
    ///          callback will be called with embxx::error::ErrorCode::Aborted
    ///          as status value.
    /// @return true in case the previous asyncWrite() operation was really
    ///         cancelled, false in case there was no unfinished asynchronous
    ///         write request.
    bool cancelWrite();

private:
    void initRead(
        CharType* buf,
        std::size_t size,
        bool performingReadUntil,
        CharType untilChar);

    void initWrite(
        const CharType* buf,
        std::size_t size);

    void canReadInterruptHandler();
    void canWriteInterruptHandler();
    void readCompleteInterruptHandler(const embxx::error::ErrorStatus& es);
    void writeCompleteInterruptHandler(const embxx::error::ErrorStatus& es);
    void invokeReadHandler(const embxx::error::ErrorStatus& es, bool interruptCtx);
    void invokeWriteHandler(const embxx::error::ErrorStatus& es, bool interruptCtx);

    Device& device_;
    EventLoop& el_;

    // Read section
    CharType* readBufStart_;
    CharType* readBufCurrent_;
    std::size_t readBufSize_;
    ReadHandler readHandler_;
    bool performingReadUntil_;
    CharType untilChar_;

    // Write section
    const CharType* writeBufStart_;
    const CharType* writeBufCurrent_;
    std::size_t writeBufSize_;
    WriteHandler writeHandler_;
};

/// @}

// Implementation
template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::Character(
    Device& device,
    EventLoop& el)
    : device_(device),
      el_(el),
      readBufStart_(nullptr),
      readBufCurrent_(nullptr),
      readBufSize_(0),
      performingReadUntil_(false),
      untilChar_(static_cast<CharType>(0)),
      writeBufStart_(nullptr),
      writeBufCurrent_(nullptr),
      writeBufSize_(0)
{
    device_.setCanReadHandler(
        std::bind(&Character::canReadInterruptHandler, this));
    device_.setCanWriteHandler(
        std::bind(&Character::canWriteInterruptHandler, this));
    device_.setReadCompleteHandler(
        std::bind(&Character::readCompleteInterruptHandler, this, std::placeholders::_1));
    device_.setWriteCompleteHandler(
        std::bind(&Character::writeCompleteInterruptHandler, this, std::placeholders::_1));

    GASSERT(!readHandler_); // No read in progress
    GASSERT(!writeHandler_); // No write in progress
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
typename Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::Device&
Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::device()
{
    return device_;
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
typename Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::EventLoop&
Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::eventLoop()
{
    return el_;
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
template <typename TFunc>
void Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::asyncRead(
    CharType* buf,
    std::size_t size,
    TFunc&& func)
{
    GASSERT(!readHandler_); // No read in progress
    readHandler_ = std::forward<TFunc>(func);
    initRead(buf, size, false, static_cast<CharType>(0));
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
template <typename TFunc>
void Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::asyncReadUntil(
    CharType* buf,
    std::size_t size,
    CharType untilChar,
    TFunc&& func)
{
    GASSERT(!readHandler_); // No read in progress
    readHandler_ = std::forward<TFunc>(func);
    initRead(buf, size, true, untilChar);
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
bool Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::cancelRead()
{
    if (!readHandler_) {
        return false;
    }

    if (!device_.cancelRead()) {
        return false;
    }

    GASSERT(readBufCurrent_ < (readBufStart_ + readBufSize_));
    invokeReadHandler(embxx::error::ErrorCode::Aborted, false);
    return true;
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
template <typename TFunc>
void Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::asyncWrite(
    const CharType* buf,
    std::size_t size,
    TFunc&& func)
{
    GASSERT(!writeHandler_); // No write in progress
    writeHandler_ = std::forward<TFunc>(func);
    initWrite(buf, size);
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
bool Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::cancelWrite()
{
    if (!writeHandler_) {
        return false;
    }

    if (!device_.cancelWrite()) {
        return false;
    }

    GASSERT(writeBufCurrent_ < (writeBufStart_ + writeBufSize_));
    invokeWriteHandler(embxx::error::ErrorCode::Aborted, false);
    return true;
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
void Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::initRead(
    CharType* buf,
    std::size_t size,
    bool performingReadUntil,
    CharType untilChar)
{
    if (size == 0) {
        invokeReadHandler(embxx::error::ErrorCode::Success, false);
        return;
    }

    readBufStart_ = buf;
    readBufCurrent_ = buf;
    readBufSize_ = size;
    performingReadUntil_ = performingReadUntil;
    untilChar_ = untilChar;
    device_.startRead(size);
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
void Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::initWrite(
    const CharType* buf,
    std::size_t size)
{
    if (size == 0) {
        invokeWriteHandler(embxx::error::ErrorCode::Success, false);
        return;
    }

    writeBufStart_ = buf;
    writeBufCurrent_ = buf;
    writeBufSize_ = size;
    device_.startWrite(size);
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
void Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::canReadInterruptHandler()
{
    while(device_.canRead()) {
        if ((readBufStart_ + readBufSize_) <= readBufCurrent_) {
            // The device control object mustn't allow it.
            GASSERT(0);
            break;
        }

        auto ch = device_.read();
        *readBufCurrent_ = ch;
        ++readBufCurrent_;

        if ((performingReadUntil_) && (ch == untilChar_)) {
            if (device_.cancelReadInterruptCtx()) {
                invokeReadHandler(embxx::error::ErrorCode::Success, true);
            }
        }
    }
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
void Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::canWriteInterruptHandler()
{
    while(device_.canWrite()) {
        if ((writeBufStart_ + writeBufSize_) <= writeBufCurrent_) {
            // The device control object mustn't allow it.
            GASSERT(0);
            break;
        }

        device_.write(*writeBufCurrent_);
        ++writeBufCurrent_;
    }
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
void Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::readCompleteInterruptHandler(
    const embxx::error::ErrorStatus& es)
{
    invokeReadHandler(es, true);
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
void Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::writeCompleteInterruptHandler(
    const embxx::error::ErrorStatus& es)
{
    invokeWriteHandler(es, true);
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
void Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::invokeReadHandler(
    const embxx::error::ErrorStatus& es,
    bool interruptCtx)
{
    GASSERT(readHandler_);
    bool postResult = false;
    auto reportedSize = static_cast<std::size_t>(readBufCurrent_ - readBufStart_);
    if (interruptCtx) {
        postResult =
            el_.postInterruptCtx(
                std::bind(
                    std::move(readHandler_),
                    es,
                    reportedSize));
    }
    else {
        postResult =
            el_.post(
                std::bind(
                    std::move(readHandler_),
                    es,
                    reportedSize));
    }

    static_cast<void>(postResult);
    GASSERT(postResult);
    GASSERT(!readHandler_);
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
void Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::invokeWriteHandler(
    const embxx::error::ErrorStatus& es,
    bool interruptCtx)
{
    GASSERT(writeHandler_);
    bool postResult = false;
    auto reportedSize = static_cast<std::size_t>(writeBufCurrent_ - writeBufStart_);
    if (interruptCtx) {
        postResult =
            el_.postInterruptCtx(
                std::bind(
                    std::move(writeHandler_),
                    es,
                    reportedSize));
    }
    else {
        postResult =
            el_.post(
                std::bind(
                    std::move(writeHandler_),
                    es,
                    reportedSize));
    }
    static_cast<void>(postResult);
    GASSERT(postResult);
    GASSERT(!writeHandler_);
}

}  // namespace driver

}  // namespace embxx




