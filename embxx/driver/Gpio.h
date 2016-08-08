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

/// @file embxx/driver/Gpio.h
/// The file contains definition of "Gpio" device driver class.


#pragma once

#include <array>
#include <functional>

#include "embxx/device/context.h"
#include "embxx/util/StaticFunction.h"
#include "embxx/util/Assert.h"
#include "embxx/util/ScopeGuard.h"
#include "embxx/error/ErrorStatus.h"

namespace embxx
{

namespace driver
{

/// @ingroup driver
/// @brief GPIO device driver
/// @details Manages the gpio lines monitoring requests and dispatches callbacks
///          to be executed in non-interrupt context using event loop.
/// @tparam TDevice Platform specific GPIO device (peripheral) control class. It
///         must expose the following interface:
///         @code
///         // Define define type for identifying gpio pins.
///         typedef std::uint8_t PinIdType;
///
///         // Set the GPIO input interrupt callback which has "void (PinIdType, bool)"
///         // signature, where the first parameter identifies the input line and
///         // second parameter specifies the value of the input line after
///         // the interrupt occurred.
///         template <typename TFunc>
///         void setHandler(TFunc&& func);
///
///         // Start gpio monitoring, i.e. enable GPIO interrupts globally if
///         // needed
///         void start(embxx::device::context::EventLoop context);
///
///         // Cancel gpio monitoring, i.e. disable GPIO interrupts globally if
///         // needed. Return true in case the operation is successful,
///         // false otherwise (the monitoring is already cancelled or wasn't
///         // started).
///         bool cancel(embxx::device::context::EventLoop context);
///
///         // Enable/Disable GPIO changes report on specific input line.
///         void setEnabled(PinIdType id, bool enabled, embxx::device::context::EventLoop context);
///
///         // Suspend report of all changes in GPIO input lines. Return true
///         // in case the suspension is successful, false otherwise.
///         bool suspend(embxx::device::context::EventLoop context);
///
///         // Resume previously suspended GPIO changes reports.
///         void(embxx::device::context::EventLoop context);
///         @endcode
/// @tparam TEventLoop Event loop class, must provide the following API member
///         functions:
///         @code
///         // Post new functor object for execution in event loop. The function
///         // is called from event loop (non-interrupt) context. Return true
///         // if the operation is successful.
///         template <typename TFunc>
///         bool post(TFunc&& func);
///
///         // Post new functor object for execution in event loop. The function
///         // is called from event loop (non-interrupt) context. Return true
///         // if the operation is successful.
///         template <typename TFunc>
///         bool postInterruptCtx(TFunc&& func)
///         @endcode
/// @tparam TNumOfLines Compile time constant specifying number of GPIO
///         lines this object must support.
/// @tparam THandler Callback storage type, must be either std::function or
///         embxx::util::StaticFunction and expose
///         "void (const std::error::ErrorStatus& es, bool value)" signature.
/// @headerfile embxx/driver/Gpio.h
template <typename TDevice,
          typename TEventLoop,
          std::size_t TNumOfLines,
          typename THandler =
              embxx::util::StaticFunction<void (const embxx::error::ErrorStatus&, bool)> >
class Gpio
{
    typedef embxx::device::context::EventLoop EventLoopCtx;
    typedef embxx::device::context::Interrupt InterruptCtx;

public:
    /// @brief Type of the Device object.
    typedef TDevice Device;

    /// @brief Type of the Event Loop object
    typedef TEventLoop EventLoop;

    /// @brief Number of supported GPIO lines.
    static const std::size_t NumOfLines = TNumOfLines;

    /// @brief Callback handler storage type.
    typedef THandler Handler;

    /// @brief GPIO pin identification type, provided by the Device.
    typedef typename Device::PinIdType PinIdType;


    /// @brief Constructor
    /// @param dev Reference to device (peripheral) control object
    /// @param el Reference to event loop object
    Gpio(Device& dev, EventLoop& el)
      : device_(dev),
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
                el_.postInterruptCtx(
                    std::bind(
                        iter->handler_,
                        embxx::error::ErrorCode::Success,
                        value));
                GASSERT(iter->handler_);
            });
    }

    /// @brief Copy constructor is deleted.
    Gpio(const Gpio&) = delete;

    /// @brief Move constructor is deleted.
    Gpio(Gpio&&) = delete;

    /// @brief Copy assignment operator is deleted.
    Gpio& operator=(const Gpio&) = delete;

    /// @brief Move assignment operator is deleted.
    Gpio& operator=(Gpio&&) = delete;

    /// @brief The destructor is default, generated by the compiler.
    ~Gpio() = default;

    /// @brief Get reference to device (peripheral) control object.
    Device& device()
    {
        return device_;
    }

    /// @brief Get referent to event loop object.
    EventLoop& eventLoop()
    {
        return el_;
    }

    /// @brief Continuous asynchronous read request.
    /// @details The operation returns immediately, the callback will be
    ///          called multiple times on changes of the requested GPIO input
    ///          value until it is removed using cancelReadCont().
    ///          The rising/falling edge interrupts configuration must
    ///          be done directly with the Device.
    /// @param id GPIO input line id.
    /// @param func Callback function, must have the following signature:
    ///        @code void handler(const embxx::error::ErrorStatus& es, bool value); @endcode
    /// @pre func is copy-constructible.
    /// @pre No active "read" on the specified GPIO line exists, i.e. no callback
    ///      is registered.
    template <typename TFunc>
    void asyncReadCont(PinIdType id, TFunc&& func)
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
            GASSERT(suspended);
            GASSERT(!"Overriding existing handler");
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

    /// @brief Cancel previously issues continuous asynchronous read.
    /// @details If there is no registered asynchronous continuous read
    ///          operation in progress, the call to this function will have
    ///          no effect and false will be returned. Otherwise the callback
    ///          will be called with embxx::error::ErrorCode::Aborted
    ///          as status value and will be deleted from the internal data
    ///          structures.
    /// @return true in case the operation was really
    ///         cancelled, false in case there was no unfinished asynchronous
    ///         continuous read in progress.
    bool cancelReadCont(PinIdType id)
    {
        return cancelReadContInternal(id, true);
    }

    /// @brief Cancel previously issues continuous asynchronous read.
    /// @details If there is no registered asynchronous continuous read
    ///          operation in progress, the call to this function will have
    ///          no effect and false will be returned. In case of successful
    ///          cancellation, no callback is invoked. This function is
    ///          suitable to be used in destructor of the component that
    ///          uses this driver to monitor GPIO input.
    /// @return true in case the operation was really
    ///         cancelled, false in case there was no unfinished asynchronous
    ///         continuous read in progress.
    bool cancelReadContNoCallback(PinIdType id)
    {
        return cancelReadContInternal(id, false);
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

    bool cancelReadContInternal(PinIdType id, bool invokeHandler)
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
            return false;
        }

        GASSERT(suspended);
        device_.setEnabled(id, false, EventLoopCtx());
        GASSERT(iter->handler_);
        if (invokeHandler) {
            el_.post(
                std::bind(
                    std::move(iter->handler_),
                    embxx::error::ErrorCode::Aborted,
                    false));
        }
        else {
            iter->handler_ = nullptr;
        }
        GASSERT(!iter->handler_);

        std::move(iter + 1, endIter, iter);
        --numOfHandlers_;

        if (numOfHandlers_ == 0) {
            device_.cancel(EventLoopCtx());
            guard.release();
        }
        return true;
    }

    Device& device_;
    EventLoop& el_;
    Infos infos_;
    std::size_t numOfHandlers_;
};

}  // namespace driver

}  // namespace embxx


