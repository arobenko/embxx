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
#include <chrono>
#include <mutex>

#include <boost/date_time.hpp>

#include "cxxtest/TestSuite.h"

#include "embxx/error/ErrorStatus.h"
#include "embxx/device/context.h"

#include "TestDevice.h"

namespace embxx
{

namespace device
{

namespace test
{

template <typename TLoopLock>
class TimerDevice : public TestDevice
{
public:
    typedef unsigned WaitUnitTimeType;
    typedef std::chrono::duration<WaitUnitTimeType, std::milli> WaitTimeUnitDuration;

    typedef TLoopLock LoopLock;

    TimerDevice(LoopLock& lock)
        : lock_(lock),
          timer_(io_),
          waitTime_(0),
          suspended_(true)
    {
    }

    ~TimerDevice()
    {
        {
            std::lock_guard<LoopLock> guard(lock_);
            timer_.cancel();
            suspended_ = false;
            suspendCond_.notify_all();
        }

        stopThread();
    }

    void startWait(
        WaitUnitTimeType milliseconds,
        embxx::device::context::EventLoop context)
    {
        static_cast<void>(context);
        std::lock_guard<LoopLock> guard(lock_);
        startInternal(milliseconds);
    }

    void startWait(
        WaitUnitTimeType milliseconds,
        embxx::device::context::Interrupt context)
    {
        static_cast<void>(context);
        startInternal(milliseconds);
    }

    bool cancelWait(embxx::device::context::EventLoop context)
    {
        static_cast<void>(context);
        std::lock_guard<LoopLock> guard(lock_);
        return cancelInternal();
    }

    bool suspendWait(embxx::device::context::EventLoop context)
    {
        static_cast<void>(context);
        std::lock_guard<LoopLock> guard(lock_);
        if ((!stopTime_.is_not_a_date_time()) ||
            (startTime_.is_not_a_date_time())) {
            return false;
        }
        assert(!suspended_);
        suspended_ = true;
        return true;
    }

    void resumeWait(embxx::device::context::EventLoop context)
    {
        static_cast<void>(context);
        std::lock_guard<LoopLock> guard(lock_);
        assert(suspended_);
        assert(stopTime_.is_not_a_date_time());
        suspended_ = false;
        suspendCond_.notify_all();
    }

    WaitUnitTimeType getElapsed(embxx::device::context::EventLoop context) const
    {
        static_cast<void>(context);
        std::lock_guard<LoopLock> guard(lock_);
        assert(!startTime_.is_not_a_date_time());
        assert(!stopTime_.is_not_a_date_time());
        auto duration = stopTime_ - startTime_;
        return duration.total_milliseconds();
    }

    template <typename TFunc>
    void setWaitCompleteCallback(TFunc&& func)
    {
        std::lock_guard<LoopLock> guard(lock_);
        callback_ = std::forward<TFunc>(func);
    }

private:
    void startInternal(WaitUnitTimeType milliseconds)
    {
        waitTime_ = milliseconds;
        startTime_ = boost::posix_time::microsec_clock::local_time();
        stopTime_ = boost::posix_time::not_a_date_time;
        suspended_ = false;

        assert(0 < waitTime_);
        timer_.expires_from_now(boost::posix_time::milliseconds(waitTime_));
        timer_.async_wait(
            [this](const boost::system::error_code& ec)
            {
                if (ec == boost::asio::error::operation_aborted) {
                    return;
                }

                std::unique_lock<LoopLock> guard(lock_);
                suspendCond_.wait(guard, [this]() -> bool {return !suspended_;});

                finaliseWait();

                assert(callback_);
                callback_(embxx::error::ErrorCode::Success);
            });
    }

    bool cancelInternal()
    {
        if ((!stopTime_.is_not_a_date_time()) ||
            (startTime_.is_not_a_date_time())) {
            return false;
        }

        finaliseWait();
        return true;
    }

    void finaliseWait()
    {
        suspended_ = true;
        stopTime_ = boost::posix_time::microsec_clock::local_time();
        timer_.cancel();
    }

    LoopLock& lock_;
    boost::asio::deadline_timer timer_;
    std::function<void (const embxx::error::ErrorStatus&)> callback_;
    WaitUnitTimeType waitTime_;
    boost::posix_time::ptime startTime_;
    boost::posix_time::ptime stopTime_;
    bool suspended_;
    std::condition_variable_any suspendCond_;
};


}  // namespace test

}  // namespace device

}  // namespace embxx


