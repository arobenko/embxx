//
// Copyright 2014 (C). Alex Robenko. All rights reserved.
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

#include <functional>
#include <algorithm>
#include <vector>
#include <memory>
#include <list>
#include <cstdint>
#include <map>
#include <mutex>
#include <cassert>

#include "cxxtest/TestSuite.h"

#include "embxx/error/ErrorStatus.h"
#include "embxx/device/context.h"

#include "ReadFifo.h"
#include "WriteFifo.h"
#include "TestDevice.h"

namespace embxx
{

namespace device
{

namespace test
{

template <typename TEventLoopLock,
          typename TCharType = std::uint8_t,
          std::size_t TFifoSize = 16,
          std::size_t TFifoOpDelayMs = 2>
class UartDevice : public TestDevice
{
    typedef TestDevice Base;

public:

    // Required definitions
    typedef TCharType CharType;

    // Implementation specific definitions
    typedef TEventLoopLock EventLoopLock;
    typedef ReadFifo<CharType, TFifoSize, TFifoOpDelayMs> RFifo;
    typedef WriteFifo<CharType, TFifoSize, TFifoOpDelayMs> WFifo;

    typedef typename RFifo::DataSeq ReadDataSeq;
    typedef typename WFifo::DataSeq WriteDataSeq;

    typedef std::function<void ()> CanDoOpHandler;
    typedef std::function<void (const embxx::error::ErrorStatus&)> OpCompleteHandler;

    // Creation and configuration interface

    UartDevice(EventLoopLock& lock)
        : Base(),
          lock_(lock),
          readInProgress_(false),
          remainingReadLen_(0),
          writeInProgress_(false),
          remainingWriteLen_(0),
          readFifo_(io_),
          writeFifo_(io_),
          readSuspended_(true),
          writeSuspended_(true)
    {
        readFifo_.setReadAvailableHandler(
            std::bind(
                &UartDevice::readAvailableHandler,
                this));

        writeFifo_.setWriteAvailableHandler(
            std::bind(
                &UartDevice::writeAvailableHandler,
                this));
    }

    ~UartDevice()
    {
        {
            std::lock_guard<EventLoopLock> guard(lock_);
            readFifo_.clear();
            writeFifo_.clear();
            readFifo_.setReadAvailableHandler(nullptr);
            writeFifo_.setWriteAvailableHandler(nullptr);
            readSuspended_ = false;
            readInProgress_ = false;
            readSuspendCond_.notify_all();
            writeSuspended_ = false;
            writeInProgress_ = false;
            writeSuspendCond_.notify_all();
        }
        stopThread();
    }

    void setDataToRead(const CharType* data, std::size_t size)
    {
        std::lock_guard<EventLoopLock> guard(lock_);
        assert(readFifo_.getDataToRead().empty());
        readFifo_.setDataToRead(data, size);
    }

    const WriteDataSeq& getWrittenData() const
    {
        std::lock_guard<EventLoopLock> guard(lock_);
        return writeFifo_.getWrittenData();
    }

    void clearWrittenData()
    {
        std::lock_guard<EventLoopLock> guard(lock_);
        writeFifo_.clear();
    }

    // Required interface
    template <typename TFunc>
    void setCanReadHandler(TFunc&& func)
    {
        std::lock_guard<EventLoopLock> guard(lock_);
        canReadHandler_ = std::forward<TFunc>(func);
    }

    template <typename TFunc>
    void setCanWriteHandler(TFunc&& func)
    {
        std::lock_guard<EventLoopLock> guard(lock_);
        canWriteHandler_ = std::forward<TFunc>(func);
    }

    template <typename TFunc>
    void setReadCompleteHandler(TFunc&& func)
    {
        std::lock_guard<EventLoopLock> guard(lock_);
        readCompleteHandler_ = std::forward<TFunc>(func);
    }

    template <typename TFunc>
    void setWriteCompleteHandler(TFunc&& func)
    {
        std::lock_guard<EventLoopLock> guard(lock_);
        writeCompleteHandler_ = std::forward<TFunc>(func);
    }

    void startRead(
        std::size_t length,
        embxx::device::context::EventLoop context)
    {
        static_cast<void>(context);
        std::lock_guard<EventLoopLock> guard(lock_);
        startReadInternal(length);
    }

    void startRead(
        std::size_t length,
        embxx::device::context::Interrupt context)
    {
        static_cast<void>(context);
        startReadInternal(length);
    }

    bool cancelRead(
        embxx::device::context::EventLoop context)
    {
        static_cast<void>(context);
        std::lock_guard<EventLoopLock> guard(lock_);
        return cancelReadInternal();
    }

    bool cancelRead(
        embxx::device::context::Interrupt context)
    {
        static_cast<void>(context);
        return cancelReadInternal();
    }

    void startWrite(
        std::size_t length,
        embxx::device::context::EventLoop context)
    {
        static_cast<void>(context);
        std::lock_guard<EventLoopLock> guard(lock_);
        startWriteInternal(length);
    }

    void startWrite(
        std::size_t length,
        embxx::device::context::Interrupt context)
    {
        static_cast<void>(context);
        startWriteInternal(length);
    }

    bool cancelWrite(
        embxx::device::context::EventLoop context)
    {
        static_cast<void>(context);
        std::lock_guard<EventLoopLock> guard(lock_);
        return cancelWriteInternal();
    }

    bool suspend(embxx::device::context::EventLoop context)
    {
        static_cast<void>(context);
        std::lock_guard<EventLoopLock> guard(lock_);
        return suspendInternal();
    }

    bool suspend(embxx::device::context::Interrupt context)
    {
        static_cast<void>(context);
        return suspendInternal();
    }

    void resume(embxx::device::context::EventLoop context)
    {
        static_cast<void>(context);
        std::lock_guard<EventLoopLock> guard(lock_);
        resumeInternal();
    }

    void resume(embxx::device::context::Interrupt context)
    {
        static_cast<void>(context);
        resumeInternal();
    }

    bool canRead(embxx::device::context::Interrupt context)
    {
        static_cast<void>(context);
        return (readFifo_.canRead() && (0 < remainingReadLen_) && readInProgress_);
    }

    bool canWrite(embxx::device::context::Interrupt context)
    {
        static_cast<void>(context);
        return (writeFifo_.canWrite() && (0 < remainingWriteLen_) && writeInProgress_);
    }

    CharType read(embxx::device::context::Interrupt context)
    {
        static_cast<void>(context);
        assert(canRead(context));
        --remainingReadLen_;
        return readFifo_.read();
    }

    void write(
        CharType value,
        embxx::device::context::Interrupt context)
    {
        static_cast<void>(context);
        assert(canWrite(context));
        writeFifo_.write(value);
        --remainingWriteLen_;
    }

private:

    void startReadInternal(
        std::size_t length)
    {
        assert(!readInProgress_);
        assert(canReadHandler_);
        assert(readCompleteHandler_);
        remainingReadLen_ = length;
        readSuspended_ = false;
        readInProgress_ = true;
        readFifo_.startRead();
    }

    bool cancelReadInternal()
    {
        if (readInProgress_) {
            finaliseRead();
            return true;
        }

        return false;
    }

    void readAvailableHandler()
    {
        std::unique_lock<EventLoopLock> guard(lock_);
        readSuspendCond_.wait(guard, [this]() -> bool {return !readSuspended_;});

        if (!canRead(embxx::device::context::Interrupt())) {
            return;
        }

        assert(canReadHandler_);
        canReadHandler_();

        if ((remainingReadLen_ == 0) && (readInProgress_)) {
            assert(readCompleteHandler_);
            finaliseRead();
            readCompleteHandler_(embxx::error::ErrorCode::Success);
        }
    }

    void finaliseRead()
    {
        readSuspended_ = true;
        readInProgress_ = false;
        readFifo_.stopRead();
    }

    void startWriteInternal(
        std::size_t length)
    {
        assert(!writeInProgress_);
        assert(canWriteHandler_);
        assert(writeCompleteHandler_);
        remainingWriteLen_ = length;
        writeSuspended_ = false;
        writeInProgress_ = true;
        writeFifo_.startWrite();
    }

    bool cancelWriteInternal()
    {
        if (writeInProgress_) {
            finaliseWrite();
            return true;
        }

        return false;
    }

    void writeAvailableHandler()
    {
        std::unique_lock<EventLoopLock> guard(lock_);
        writeSuspendCond_.wait(guard, [this]() -> bool {return !writeSuspended_;});

        if (canWrite(embxx::device::context::Interrupt())) {
            assert(canWriteHandler_);
            canWriteHandler_();
        }

        if ((remainingWriteLen_ == 0) && writeInProgress_ && (writeFifo_.complete())) {
            assert(writeCompleteHandler_);
            finaliseWrite();
            writeCompleteHandler_(embxx::error::ErrorCode::Success);
        }
    }

    void finaliseWrite()
    {
        writeFifo_.stopWrite();
        writeSuspended_ = true;
        writeInProgress_ = false;
    }

    bool suspendInternal()
    {
        bool suspended = readInProgress_ || writeInProgress_;
        GASSERT((!readInProgress_) || (!readSuspended_)); // If in progress, not suspended
        GASSERT((!writeInProgress_) || (!writeSuspended_));
        readSuspended_ = true;
        writeSuspended_ = true;
        return suspended;
    }

    void resumeInternal() {
        GASSERT(readInProgress_ || writeInProgress_);
        if (readInProgress_) {
            readSuspended_ = false;
        }

        if (writeInProgress_) {
            writeSuspended_ = false;
        }
    }

    EventLoopLock& lock_;
    bool readInProgress_;
    std::size_t remainingReadLen_;
    bool writeInProgress_;
    std::size_t remainingWriteLen_;
    RFifo readFifo_;
    WFifo writeFifo_;
    CanDoOpHandler canReadHandler_;
    CanDoOpHandler canWriteHandler_;
    OpCompleteHandler readCompleteHandler_;
    OpCompleteHandler writeCompleteHandler_;
    volatile bool readSuspended_;
    std::condition_variable_any readSuspendCond_;
    volatile bool writeSuspended_;
    std::condition_variable_any writeSuspendCond_;
};

}  // namespace test

}  // namespace device

}  // namespace embxx



