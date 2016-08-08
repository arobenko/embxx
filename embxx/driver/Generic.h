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

/// @file embxx/driver/Generic.h
/// Implements "Generic" driver that forwards data from interrupt handler
/// to be processed in the "regular" thread context.

#pragma once

#include "embxx/util/StaticFunction.h"
#include "embxx/util/Assert.h"
#include "embxx/error/ErrorStatus.h"

namespace embxx
{

namespace driver
{

/// @addtogroup driver
/// @{

/// @brief Declaration of Generic driver.
/// @details The class doesn't have a body, see specialisation.
/// @headerfile embxx/driver/Generic.h
template <typename TDevice,
          typename TEventLoop,
          typename TSignature = void (const embxx::error::ErrorStatus&),
          typename THandler = embxx::util::StaticFunction<TSignature> >
class Generic;

/// @brief Generic driver.
/// @details Provides a facility to forward data from interrupt context
///          to be processed in the "regular" execution thread. It is useful to
///          process non-critical data, such as button press, in non-interrupt
///          context. This is the template specialisation of the following
///          class declaration:
///          @code
///          template <typename TDevice,
///                    typename TEventLoop,
///                    typename TSignature = void (const embxx::error::ErrorStatus&),
///                    typename THandler = embxx::util::StaticFunction<TSignature> >
///          class Generic;
///          @endcode
///          Please pay attention that function signature in THandler type
///          must be the same as third template parameter.
/// @tparam TDevice Low level peripheral control device. Must provide member
///         function with following declaration to enable setting of handler
///         to be executed in interrupt context:
///         @code
///         template <typename TFunc>
///         void setHandler(... /* up to 3 args */, TFunc&& func);
///         @endcode
/// @tparam TEventLoop Event loop type - a variant of embxx::util::EventLoop.
/// @tparam THandler Type to store handler to be executed in "regular" thread
///         context. Must be either std::function or embxx::util::StaticFunction
/// @tparam TArgs Types of other arguments passed by/to callbacks.
/// @headerfile embxx/driver/Generic.h
template <typename TDevice,
          typename TEventLoop,
          typename THandler,
          typename... TArgs>
class Generic<TDevice, TEventLoop, void(TArgs...), THandler>
{
public:
    /// @brief Device (peripheral) control object type
    typedef TDevice Device;

    /// @brief Evnet loop type
    typedef TEventLoop EventLoop;

    /// @brief Handler type
    typedef THandler Handler;

    /// @brief Constructor
    /// @param dev Reference to device (peripheral) control object.
    /// @param el Reference to event loop object.
    Generic(Device& dev, EventLoop& el);

    /// @brief Get reference to device (peripheral) control object.
    Device& device();

    /// @brief Get referent to event loop object.
    EventLoop& eventLoop();

    /// @brief Set handler
    /// @details The handler will be executed in the "regular" execution thread
    ///          in the event loop processing.
    /// @param[in] func Functor to be executed. Must receive the same parameters
    ///            as reported by the device control object.
    template <typename TFunc>
    void setHandler(TFunc&& func);

    /// @brief Set handler
    /// @details The handler will be executed in the "regular" execution thread
    ///          in the event loop processing.
    /// @param[in] arg1 First argument to be passed to setHandler of the device
    ///            class.
    /// @param[in] func Functor to be executed. Must receive the same parameters
    ///            as reported by the device control object.
    template <typename TArg1, typename TFunc>
    void setHandler(TArg1&& arg1, TFunc&& func);

    /// @brief Set handler
    /// @details The handler will be executed in the "regular" execution thread
    ///          in the event loop processing.
    /// @param[in] arg1 First argument to be passed to setHandler of the device
    ///            class.
    /// @param[in] arg2 Second argument to be passed to setHandler of the device
    ///            class.
    /// @param[in] func Functor to be executed. Must receive the same parameters
    ///            as reported by the device control object.
    template <typename TArg1, typename TArg2, typename TFunc>
    void setHandler(TArg1&& arg1, TArg2&&, TFunc&& func);

    /// @brief Set handler
    /// @details The handler will be executed in the "regular" execution thread
    ///          in the event loop processing.
    /// @param[in] arg1 First argument to be passed to setHandler of the device
    ///            class.
    /// @param[in] arg2 Second argument to be passed to setHandler of the device
    ///            class.
    /// @param[in] arg3 Third argument to be passed to setHandler of the device
    ///            class.
    /// @param[in] func Functor to be executed. Must receive the same parameters
    ///            as reported by the device control object.
    template <typename TArg1, typename TArg2, typename TArg3, typename TFunc>
    void setHandler(TArg1&& arg1, TArg2&& arg2, TArg3&& arg3, TFunc&& func);

private:
    Device& device_;
    EventLoop& el_;
    Handler handler_;
};

/// @}

// Implementation
template <typename TDevice,
          typename TEventLoop,
          typename THandler,
          typename... TArgs>
Generic<TDevice, TEventLoop, void(TArgs...), THandler>::Generic(
    Device& dev,
    EventLoop& el)
    : device_(dev),
      el_(el)
{
}

template <typename TDevice,
          typename TEventLoop,
          typename THandler,
          typename... TArgs>
typename Generic<TDevice, TEventLoop, void(TArgs...), THandler>::Device&
Generic<TDevice, TEventLoop, void(TArgs...), THandler>::device()
{
    return device_;
}

template <typename TDevice,
          typename TEventLoop,
          typename THandler,
          typename... TArgs>
typename Generic<TDevice, TEventLoop, void(TArgs...), THandler>::EventLoop&
Generic<TDevice, TEventLoop, void(TArgs...), THandler>::eventLoop()
{
    return el_;
}

template <typename TDevice,
          typename TEventLoop,
          typename THandler,
          typename... TArgs>
template <typename TFunc>
void Generic<TDevice, TEventLoop, void(TArgs...), THandler>::setHandler(
    TFunc&& func)
{
    handler_ = std::forward<TFunc>(func);
    device_.setHandler(
        [this](TArgs... args)
        {
            if (handler_) {
                auto result = el_.postInterruptCtx(
                    std::bind(handler_, args...));
                GASSERT(result);
                static_cast<void>(result);
            }
        });
}

template <typename TDevice,
          typename TEventLoop,
          typename THandler,
          typename... TArgs>
template <typename TArg1, typename TFunc>
void Generic<TDevice, TEventLoop, void(TArgs...), THandler>::setHandler(
    TArg1&& arg1,
    TFunc&& func)
{
    handler_ = std::forward<TFunc>(func);
    device_.setHandler(
        std::forward<TArg1>(arg1),
        [this](TArgs... args)
        {
            if (handler_) {
                auto result = el_.postInterruptCtx(
                    std::bind(handler_, args...));
                GASSERT(result);
                static_cast<void>(result);
            }
        });
}

template <typename TDevice,
          typename TEventLoop,
          typename THandler,
          typename... TArgs>
template <typename TArg1, typename TArg2, typename TFunc>
void Generic<TDevice, TEventLoop, void(TArgs...), THandler>::setHandler(
    TArg1&& arg1,
    TArg2&& arg2,
    TFunc&& func)
{
    handler_ = std::forward<TFunc>(func);
    device_.setHandler(
        std::forward<TArg1>(arg1),
        std::forward<TArg2>(arg2),
        [this](TArgs... args)
        {
            if (handler_) {
                auto result = el_.postInterruptCtx(
                    std::bind(handler_, args...));
                GASSERT(result);
                static_cast<void>(result);
            }
        });
}

template <typename TDevice,
          typename TEventLoop,
          typename THandler,
          typename... TArgs>
template <typename TArg1, typename TArg2, typename TArg3, typename TFunc>
void Generic<TDevice, TEventLoop, void(TArgs...), THandler>::setHandler(
    TArg1&& arg1,
    TArg2&& arg2,
    TArg3&& arg3,
    TFunc&& func)
{
    handler_ = std::forward<TFunc>(func);
    device_.setHandler(
        std::forward<TArg1>(arg1),
        std::forward<TArg2>(arg2),
        std::forward<TArg3>(arg3),
        [this](TArgs... args)
        {
            if (handler_) {
                auto result = el_.postInterruptCtx(
                    std::bind(handler_, args...));
                GASSERT(result);
                static_cast<void>(result);
            }
        });
}

}  // namespace driver

}  // namespace embxx


