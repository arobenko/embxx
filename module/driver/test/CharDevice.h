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

#include <functional>
#include <thread>
#include <algorithm>
#include <string>
#include <memory>
#include <list>

#include <boost/asio.hpp>
#include <boost/date_time.hpp>

#include "cxxtest/TestSuite.h"

#include "CommonEventLoopDefs.h"

class CharDevice
{
public:
    CharDevice(LoopLock& lock)
        : lock_(lock),
          work_(io_),
          timer_(io_),
          readInterruptEnabled_(false),
          writeInterruptEnabled_(false),
          maxBufSize_(5),
          delayMs_(10),
          readLen_(0),
          writeLen_(0)
    {

        thread_.reset(
            new std::thread(
                [this]()
                {
                    try {
                        io_.run();
                    }
                    catch (const std::exception& e)
                    {
                        TS_FAIL(e.what());
                        assert(!"Mustn't happen");
                    }
                }));
    }

    ~CharDevice()
    {
        if (thread_) {
            stop();
        }
    }

    void start() {
        programWait();
    }

    void stop() {
        io_.stop();
        thread_->join();
        thread_.reset();
        std::for_each(writeBuf_.begin(), writeBuf_.end(),
            [this](char ch) { writeString_ += ch; });
    }

    void setBufSize(unsigned size)
    {
        TS_ASSERT(0 < size);
        maxBufSize_ = size;
    }

    void setOpDelay(unsigned ms)
    {
        TS_ASSERT(0 < ms);
        delayMs_ = ms;
    }

    template <typename TString>
    void setReadString(TString&& str)
    {
        readString_ = std::forward<TString>(str);
    }

    const std::string& getWriteString() const
    {
        return writeString_;
    }

    void clearWriteString()
    {
        writeString_.clear();
    }

    // Required interface
    typedef char CharType;

    void startRead(std::size_t length) {
        std::lock_guard<LoopLock> guard(lock_);
        TS_ASSERT(0 < length);
        TS_ASSERT_EQUALS(readLen_, 0);
        readLen_ = length;
        readInterruptEnabled_ = true;
    }

    bool cancelRead() {
        std::lock_guard<LoopLock> guard(lock_);
        return cancelReadInterruptCtx();
    }

    bool cancelReadInterruptCtx() {
        readInterruptEnabled_ = false;
        bool result = (readLen_ != 0);
        readLen_ = 0;
        return result;
    }


    void startWrite(std::size_t length) {
        std::lock_guard<LoopLock> guard(lock_);
        TS_ASSERT(0 < length);
        TS_ASSERT_EQUALS(writeLen_, 0);
        writeLen_ = length;
        writeInterruptEnabled_ = true;
    }

    bool cancelWrite() {
        std::lock_guard<LoopLock> guard(lock_);
        writeInterruptEnabled_ = false;
        bool result = (writeLen_ != 0);
        writeLen_ = 0;
        return result;
    }

    bool canRead() const
    {
        return ((!readBuf_.empty()) && (0 < readLen_));
    }

    bool canWrite() const
    {
        return ((writeBuf_.size() < maxBufSize_) && (0 < writeLen_));
    }

    CharType read() {
        TS_ASSERT(canRead());
        auto ch = readBuf_.front();
        readBuf_.pop_front();
        --readLen_;
        return ch;
    }

    void write(CharType value)
    {
        TS_ASSERT(canWrite());
        writeBuf_.push_back(value);
        --writeLen_;
    }

    template <typename TFunc>
    void setCanReadHandler(TFunc&& func)
    {
        canReadHandler_ = std::forward<TFunc>(func);
    }

    template <typename TFunc>
    void setCanWriteHandler(TFunc&& func)
    {
        canWriteHandler_ = std::forward<TFunc>(func);
    }

    template <typename TFunc>
    void setReadCompleteHandler(TFunc&& func)
    {
        readCompleteHandler_ = std::forward<TFunc>(func);
    }

    template <typename TFunc>
    void setWriteCompleteHandler(TFunc&& func)
    {
        writeCompleteHandler_ = std::forward<TFunc>(func);
    }

private:

    void interruptHandler()
    {
        lock_.lockInterruptCtx();
        if (writeInterruptEnabled_) {
            if (!writeBuf_.empty()) {
                writeString_ += writeBuf_.front();
                writeBuf_.pop_front();
            }
            TS_ASSERT(canWrite());
            TS_ASSERT(canWriteHandler_);
            canWriteHandler_();

            if (writeLen_ == 0) {
                writeInterruptEnabled_ = false;
                TS_ASSERT(writeCompleteHandler_);
                writeCompleteHandler_(embxx::error::ErrorCode::Success);
            }
        }

        if (readInterruptEnabled_ && (!readString_.empty())) {
            readBuf_.push_back(readString_[0]);
            std::string tempReadString(readString_.begin() + 1U, readString_.end());
            readString_ = std::move(tempReadString);
            TS_ASSERT(canRead());
            TS_ASSERT(canReadHandler_);
            canReadHandler_();

            if ((readLen_ == 0) && (readInterruptEnabled_)) {
                // Not cancelled yet
                readInterruptEnabled_ = false;
                TS_ASSERT(readCompleteHandler_);
                readCompleteHandler_(embxx::error::ErrorCode::Success);
            }
        }

        lock_.unlockInterruptCtx();
    }

    void programWait()
    {
        timer_.cancel();
        timer_.expires_from_now(boost::posix_time::milliseconds(delayMs_));
        timer_.async_wait(
            [this](const boost::system::error_code& ec)
            {
                if (ec == boost::asio::error::operation_aborted) {
                    return;
                }

                interruptHandler();
                programWait();
            });
    }

    // Thread
    LoopLock& lock_;
    boost::asio::io_service io_;
    boost::asio::io_service::work work_;
    boost::asio::deadline_timer timer_;
    std::unique_ptr<std::thread> thread_;
    volatile bool readInterruptEnabled_;
    volatile bool writeInterruptEnabled_;

    // Control
    volatile unsigned maxBufSize_;
    volatile unsigned delayMs_;

    // Read section
    std::list<CharType> readBuf_;
    std::string readString_;
    std::function<void ()> canReadHandler_;
    std::function<void (const embxx::error::ErrorStatus&)> readCompleteHandler_;
    std::size_t readLen_;

    // Write section
    std::list<CharType> writeBuf_;
    std::string writeString_;
    std::function<void ()> canWriteHandler_;
    std::function<void (const embxx::error::ErrorStatus&)> writeCompleteHandler_;
    std::size_t writeLen_;
};



