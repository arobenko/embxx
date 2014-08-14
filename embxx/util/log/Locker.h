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

/// @file embxx/util/log/Locker.h
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
class Locker : public details::LoggerDecoratorBase<TNextLayer>
{
    typedef details::LoggerDecoratorBase<TNextLayer> Base;
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
    explicit Locker(LockType& lock, TParams&&... params)
      : Base(std::forward<TParams>(params)...),
        lock_(lock)
    {
    }

    /// @brief Locks the provided lock before calling begin() of the next layer.
    /// @param[in] level requested logging level
    /// @note Exception guarantee: Basic
    void begin(Level level)
    {
        lock_.lock();
        Base::begin(level);
    }

    /// @brief Unlocks the provided lock after calling end() of the next layer.
    /// @param[in] level requested logging level
    /// @note Exception guarantee: Basic
    void end(Level level)
    {
        Base::end(level);
        lock_.unlock();
    }

private:
    LockType& lock_;
};

/// @}

}  // namespace log

}  // namespace util

}  // namespace embxx
