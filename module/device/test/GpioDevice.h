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

#include <mutex>
#include <list>
#include <algorithm>

#include "embxx/device/context.h"
#include "TestDevice.h"

namespace embxx
{

namespace device
{

namespace test
{

template <typename TLoopLock>
class GpioDevice : public TestDevice
{
public:
    typedef unsigned PinIdType;
    typedef TLoopLock LoopLock;

    struct GpioInfo
    {
        PinIdType pin_;
        unsigned delayMs_;
        bool value_;
    };

    typedef std::list<GpioInfo> GpiosList;

    typedef embxx::device::context::EventLoop EventLoopCtx;

    GpioDevice(LoopLock& lock)
        : lock_(lock),
          timer_(io_),
          suspended_(false),
          running_(false)
    {
    }

    ~GpioDevice()
    {
        std::lock_guard<LoopLock> guard(lock_);
        timer_.cancel();
        suspended_ = false;
        running_ = false;
        suspendCond_.notify_all();
    }

    void programGpios(GpiosList&& gpiosList)
    {
        gpiosList_ = std::move(gpiosList);
    }

    template <typename TFunc>
    void setHandler(TFunc&& func)
    {
        handler_ = std::forward<TFunc>(func);
    }

    void start(EventLoopCtx)
    {
        std::lock_guard<LoopLock> guard(lock_);
        assert(!gpiosList_.empty());
        assert(handler_);
        assert(!running_);
        running_ = true;
        programNextWait();
    }

    void cancel(EventLoopCtx)
    {
        std::lock_guard<LoopLock> guard(lock_);
        assert(running_);
        assert(!gpiosList_.empty());
        assert(handler_);
        running_ = false;
        suspended_ = false;
        timer_.cancel();
    }

    bool suspend(EventLoopCtx)
    {
        std::lock_guard<LoopLock> guard(lock_);
        assert(!suspended_);
        if (!running_) {
            return false;
        }
        suspended_ = true;
        return true;
    }

    void resume(EventLoopCtx)
    {
        std::lock_guard<LoopLock> guard(lock_);
        assert(suspended_);
        assert(running_);
        suspended_ = false;
        suspendCond_.notify_all();
    }

    void setEnabled(PinIdType id, bool enabled, EventLoopCtx)
    {
        if (enabled) {
            assert(std::find(enabledGpios_.begin(), enabledGpios_.end(), id) == enabledGpios_.end());
            enabledGpios_.push_back(id);
            return;
        }

        auto iter = std::find(enabledGpios_.begin(), enabledGpios_.end(), id);
        if (iter == enabledGpios_.end()) {
            return;
        }
        enabledGpios_.erase(iter);
    }


private:
    void programNextWait()
    {
        assert(!gpiosList_.empty());
        auto& nextGpio = gpiosList_.front();
        timer_.expires_from_now(boost::posix_time::milliseconds(nextGpio.delayMs_));
        timer_.async_wait(
            [this, &nextGpio](const boost::system::error_code& ec)
            {
                if (ec == boost::asio::error::operation_aborted) {
                    return;
                }

                assert(!ec);
                std::unique_lock<LoopLock> guard(lock_);
                suspendCond_.wait(guard, [this]() -> bool {return !suspended_;});

                auto iter = std::find(enabledGpios_.begin(), enabledGpios_.end(), nextGpio.pin_);
                if (iter != enabledGpios_.end()) {
                    assert(handler_);
                    handler_(nextGpio.pin_, nextGpio.value_);
                }

                gpiosList_.pop_front();
                if (!gpiosList_.empty()) {
                    io_.post(std::bind(&GpioDevice::programNextWait, this));
                }
            });
    }

    LoopLock& lock_;
    boost::asio::deadline_timer timer_;
    std::function<void (PinIdType, bool)> handler_;
    volatile bool suspended_;
    bool running_;
    std::condition_variable_any suspendCond_;
    GpiosList gpiosList_;
    std::list<PinIdType> enabledGpios_;
};


}  // namespace test

}  // namespace device

}  // namespace embxx



