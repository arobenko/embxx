//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

#pragma once

#include <functional>
#include "embxx/util/StaticFunction.h"
#include "embxx/util/Assert.h"
#include "ErrorStatus.h"

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
///         // Enable "read" interrupts, i.e. interrupt when there is
///         // at least one character to read.
///         void setReadInterruptEnabled(bool enabled);
///
///         // Enable "write" interrupts, i.e. interrupt when there is
///         // space for at least one character to be written.
///         void setWriteInterruptEnabled(bool enabled);
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
///
///         // Set the "can read" interrupt callback. Expose signature "void ()".
///         // The callback must be called when there is at least one byte
///         // available for read. The callback will perform multiple canRead()
///         // and read() calls until canRead() returns false.
///         template <typename TFunc>
///         void setCanReadHandler(TFunc&& func);
///
///         // Set the "can write" interrupt callback. Expose signature "void ()".
///         // The callback must be called when there is a space for at least
///         // one byte to be written. The callback will perform multiple canWrite()
///         // and write() calls until canWrite() returns false.
///         template <typename TFunc>
///         void setCanWriteHandler(TFunc&& func);
///         @endcode
/// @tparam TEventLoop A variant of embxx::util::EventLoop object that is used
///         to execute posted handlers in regular thread context.
/// @tparam TReadHandler A function class that is supposed to store "read"
///         complete callback. Must be either std::function of embxx::util::StaticFunction
///         and provide "void(ErrorStatus, std::size_t)" calling interface,
///         where the first parameter is error status of the operation and
///         second one is how many bytes were actually read in the operation.
/// @tparam TWriteHandler A function class that is supposed to store "write"
///         complete callback. Must be either std::function of embxx::util::StaticFunction
///         and provide "void(ErrorStatus, std::size_t)" calling interface,
///         where the first parameter is error status of the operation and
///         second one is how many bytes were actually written in the operation.
/// @headerfile embxx/driver/Character.h
template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler = embxx::util::StaticFunction<void(ErrorStatus, std::size_t)>,
          typename TWriteHandler = embxx::util::StaticFunction<void(ErrorStatus, std::size_t)> >
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
    ///        @code void callback(embxx::driver::ErrorStatus status, std::size_t bytesRead); @endcode
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
    ///        @code void callback(embxx::driver::ErrorStatus status, std::size_t bytesRead);@endcode
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
    ///          callback will be called with embxx::driver::ErrorStatus::Aborted
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
    ///        @code void callback(embxx::driver::ErrorStatus status, std::size_t bytesWritten); @endcode
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
    ///          callback will be called with embxx::driver::ErrorStatus::Aborted
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
    device_.setReadInterruptEnabled(false);
    if (!readHandler_) {
        return false;
    }

    auto result = el_.post(
        std::bind(
            std::move(readHandler_),
            ErrorStatus::Success,
            static_cast<std::size_t>(readBufCurrent_ - readBufStart_)));
    static_cast<void>(result);

    GASSERT(result);
    GASSERT(!readHandler_);
    GASSERT(readBufCurrent_ < (readBufStart_ + readBufSize_));
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
    device_.setWriteInterruptEnabled(false);
    if (!writeHandler_) {
        return false;
    }

    auto result = el_.post(
        std::bind(
            std::move(writeHandler_),
            ErrorStatus::Success,
            static_cast<std::size_t>(writeBufCurrent_ - writeBufStart_)));
    static_cast<void>(result);

    GASSERT(result);
    GASSERT(!writeHandler_);
    GASSERT(writeBufCurrent_ < (writeBufStart_ + writeBufSize_));
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
        el_.post(std::bind(std::move(readHandler_), ErrorStatus::Success, size));
        GASSERT(!readHandler_);
        return;
    }
    readBufStart_ = buf;
    readBufCurrent_ = buf;
    readBufSize_ = size;
    performingReadUntil_ = performingReadUntil;
    untilChar_ = untilChar;
    device_.setReadInterruptEnabled(true);
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
        el_.post(std::bind(std::move(writeHandler_), ErrorStatus::Success, size));
        GASSERT(!writeHandler_);
        return;
    }

    writeBufStart_ = buf;
    writeBufCurrent_ = buf;
    writeBufSize_ = size;
    device_.setWriteInterruptEnabled(true);
}

template <typename TDevice,
          typename TEventLoop,
          typename TReadHandler,
          typename TWriteHandler>
void Character<TDevice, TEventLoop, TReadHandler, TWriteHandler>::canReadInterruptHandler()
{
    while(device_.canRead()) {
        GASSERT(readBufCurrent_ < (readBufStart_ + readBufSize_));
        auto ch = device_.read();
        *readBufCurrent_ = ch;
        ++readBufCurrent_;
        if (((readBufStart_ + readBufSize_) <= readBufCurrent_) ||
            ((performingReadUntil_) && (ch == untilChar_))) {
            el_.postInterruptCtx(
                std::bind(
                    std::move(readHandler_),
                    ErrorStatus::Success,
                    static_cast<std::size_t>(readBufCurrent_ - readBufStart_)));
            device_.setReadInterruptEnabled(false);
            return;
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
        GASSERT(writeBufCurrent_ < (writeBufStart_ + writeBufSize_));
        device_.write(*writeBufCurrent_);
        ++writeBufCurrent_;
        if ((writeBufStart_ + writeBufSize_) <= writeBufCurrent_) {
            el_.postInterruptCtx(
                std::bind(
                    std::move(writeHandler_),
                    ErrorStatus::Success,
                    writeBufSize_));

            device_.setWriteInterruptEnabled(false);
            return;
        }
    }
}

}  // namespace driver

}  // namespace embxx




