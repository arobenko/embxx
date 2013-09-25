//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file embxx/driver/Generic.h
/// Implements "Generic" driver that forwards data from interrupt handler
/// to be processed in the "regular" thread context.

#pragma once

#include "embxx/util/StaticFunction.h"

namespace embxx
{

namespace driver
{

/// @addtogroup driver
/// @{

template <typename TDevice,
          typename TEventLoop,
          typename TSignature = void (),
          typename THandler = embxx::util::StaticFunction<TSignature> >
class Generic;

/// @brief Generic driver.
/// @details Provides a facility to forward data from interrupt context
///          to be processed in the "regular" execution thread. It is useful to
///          process non-critical data, such as button press, in non-interrupt
///          context. This is the template specialization of the following
///          class declaration:
///          @code
///          template <typename TDevice,
///                    typename TEventLoop,
///                    typename TSignature = void (),
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
///         void setHandler(TFunc&& func);
///         @endcode
/// @tparam TEventLoop Event loop type - a variant of embxx::util::EventLoop.
/// @tparam THandler Type to store handler to be executed in "regular" thread
///         context. Must be either std::function or embxx::util::StaticFunction
/// @tparam TArgs Types of other arguments passed by/to callbacks.
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
    /// @param device Reference to device (peripheral) control object.
    /// @param el Reference to event loop object.
    Generic(Device& device, EventLoop& el);

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
    Device& device,
    EventLoop& el)
    : device_(device),
      el_(el)
{
    device_.setHandler(
        [this](TArgs... args)
        {
            if (handler_) {
                el_.postInterruptCtx(
                    std::bind(handler_, args...));
            }
        });
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
}

}  // namespace driver

}  // namespace embxx


