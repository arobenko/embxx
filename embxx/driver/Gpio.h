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

#include <array>
#include <functional>

#include "embxx/device/context.h"
#include "embxx/util/StaticFunction.h"
#include "embxx/util/Assert.h"
#include "embxx/util/ScopeGuard.h"

namespace embxx
{

namespace driver
{

template <typename TDevice,
          typename TEventLoop,
          std::size_t TNumOfLines,
          typename THandler = embxx::util::StaticFunction<void (bool)> >
class Gpio
{
    typedef embxx::device::context::EventLoop EventLoopCtx;
    typedef embxx::device::context::Interrupt InterruptCtx;

public:
    typedef TDevice Device;

    typedef TEventLoop EventLoop;

    static const std::size_t NumOfLines = TNumOfLines;

    typedef THandler Handler;

    typedef typename Device::PinIdType PinIdType;

    Gpio(Device& device, EventLoop& el)
      : device_(device),
        el_(el),
        numOfHandlers_(0)
    {
        device_.setHandler(
            [this](PinIdType id, bool value)
            {
                auto endIter = infos_.begin() + numOfHandlers_;
                auto iter = lowerBound(id, endIter);
                if ((iter == endIter) ||
                    (iter->id_ != id)) {
                    GASSERT(!"Spurious GPIO interrupt");
                    return;
                }

                GASSERT(iter->handler_);
                el_.postInterruptCtx(std::bind(iter->handler_, value));
                GASSERT(iter->handler_);
            });
    }

    Gpio(const Gpio&) = delete;

    Gpio(Gpio&&) = delete;

    Gpio& operator=(const Gpio&) = delete;

    Gpio& operator=(Gpio&&) = delete;

    ~Gpio() = default;

    Device& device()
    {
        return device_;
    }

    EventLoop& eventLoop()
    {
        return el_;
    }

    /// @pre func is copy-constructible
    template <typename TFunc>
    void setHandler(PinIdType id, TFunc&& func)
    {
        bool suspended = device_.suspend(EventLoopCtx());
        auto guard = embxx::util::makeScopeGuard(
            [this, suspended]()
            {
                if (suspended) {
                    device_.resume(EventLoopCtx());
                }
            });

        auto endIter = infos_.begin() + numOfHandlers_;
        auto iter = lowerBound(id, endIter);

        if ((iter != endIter) &&
            (iter->id_ == id)) {
            iter->handler_ = std::forward<TFunc>(func);
            GASSERT(suspended);
            return;
        }

        GASSERT(numOfHandlers_ < NumOfLines);
        std::move_backward(iter, endIter, endIter + 1);
        iter->id_ = id;
        iter->handler_ = std::forward<TFunc>(func);
        ++numOfHandlers_;

        device_.setEnabled(id, true, EventLoopCtx());

        if (!suspended) {
            device_.start(EventLoopCtx());
        }
    }

    void setHandler(PinIdType id, std::nullptr_t)
    {
        bool suspended = device_.suspend(EventLoopCtx());
        auto guard = embxx::util::makeScopeGuard(
            [this, suspended]()
            {
                if (suspended) {
                    device_.resume(EventLoopCtx());
                }
            });

        auto endIter = infos_.begin() + numOfHandlers_;
        auto iter = lowerBound(id, endIter);

        if ((iter == endIter) ||
            (iter->id_ != id)) {
            return;
        }

        GASSERT(suspended);
        device_.setEnabled(id, false, EventLoopCtx());
        std::move(iter + 1, endIter, iter);
        --numOfHandlers_;

        if (numOfHandlers_ == 0) {
            device_.cancel(EventLoopCtx());
            guard.release();
            return;
        }
    }

private:
    struct Node
    {
        Node() : id_(PinIdType()) {}

        PinIdType id_;
        Handler handler_;
    };

    typedef std::array<Node, NumOfLines> Infos;

    auto lowerBound(PinIdType id, typename Infos::iterator endIter) -> decltype(endIter)
    {
        return
            std::lower_bound(infos_.begin(), endIter, id,
                [](const Node& n, PinIdType i)->bool
                {
                    return n.id_ < i;
                });
    }

    Device& device_;
    EventLoop& el_;
    Infos infos_;
    std::size_t numOfHandlers_;
};

}  // namespace driver

}  // namespace embxx


