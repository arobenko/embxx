//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file util/log/Locker.h
/// Contains definition of Locker decorator of the StreamLogger.

#pragma once

#include "LoggerDecoratorBase.h"

namespace embxx
{

namespace util
{

namespace log
{

/// @addtogroup util
/// @{

/// @brief Locking decorator for the stream output.
/// @details Locks the provided locking facility when begin() is called
///          and unlocks it when end() is called.
/// @tparam TLock Lock type (usually std::mutex_t>
/// @tparam TNextLayer Next layer
/// @headerfile embxx/util/log/Locker.h
template<typename TLock, typename TNextLayer>
class Locker : public LoggerDecoratorBase<TNextLayer>
{
    typedef LoggerDecoratorBase<TNextLayer> Base;
public:
    /// Lock type
    typedef TLock LockType;

    /// @brief Constructor
    /// @details Keeps reference to provided lock and forwards all
    ///          the other parameters to the constructor of the next layer.
    /// @param[in, out] lock reference to lock
    /// @param[in] params Zero or more parameters to be forwarded to the next layer.
    /// @note Exception guarantee: Strong
    template<typename... TParams>
    Locker(LockType& lock, TParams&&... params);

    /// @brief Locks the provided lock before calling begin() of the next layer.
    /// @param[in] level requested logging level
    /// @note Exception guarantee: Basic
    void begin(Level level);

    /// @brief Unlocks the provided lock after calling end() of the next layer.
    /// @param[in] level requested logging level
    /// @note Exception guarantee: Basic
    void end(Level level);
private:
    LockType& lock_;
};

/// @}

// Implementation
template<typename TLock, typename TNextLayer>
template<typename... TParams>
Locker<TLock, TNextLayer>::Locker(LockType& lock, TParams&&... params)
    : Base(std::forward<TParams>(params)...),
      lock_(lock)
{
}

template<typename TLock, typename TNextLayer>
void Locker<TLock, TNextLayer>::begin(Level level)
{
    lock_.lock();
    Base::begin(level);
}

template<typename TLock, typename TNextLayer>
void Locker<TLock, TNextLayer>::end(Level level)
{
    Base::end(level);
    lock_.unlock();
}

}  // namespace log

}  // namespace util

}  // namespace embxx
