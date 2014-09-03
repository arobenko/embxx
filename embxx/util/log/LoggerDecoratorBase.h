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

/// @file embxx/util/log/LoggerDecoratorBase.h
/// Contains definitions of the Base class for every defined decorator
/// of the StreamLogger.

#pragma once

#include <utility>
#include "LogLevel.h"

namespace embxx
{

namespace util
{

namespace log
{

namespace details
{

/// @brief Base class for all defined decorators of the StraemLogger
/// @details It contains instance of the next layer and redirects
///          begin/end requests to it.
/// @tparam TNextLayer Next layer type
/// @headerfile embxx/util/log/LoggerDecoratorBase.h
template <typename TNextLayer>
class LoggerDecoratorBase
{
public:
    /// @brief Type of the next layer
    typedef TNextLayer NextLayer;

    /// @brief Type of the output stream object
    typedef typename NextLayer::Stream Stream;

    /// @brief MinLevel is the same as defined in the next layer
    static const Level MinLevel = TNextLayer::MinLevel;

    /// @brief Constructor
    /// @details Forwards all the parameters to the constructor of the next layer
    /// @param[in] params Zero or more parameters to be forwarded to the next layer.
    /// @note Exception guarantee: Strong
    template<typename... TParams>
    explicit LoggerDecoratorBase(TParams&&... params);

    /// @brief Returns reference to output stream reported by the next layer.
    /// @return reference to output stream
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    inline Stream& stream();

    /// @brief Calls "begin()" member function of the next layer.
    /// @param[in] level requested logging level
    inline void begin(Level level);

    /// @brief Calls "end()" member function of the next layer.
    /// @param[in] level requested logging level
    inline void end(Level level);
private:
    NextLayer nextLayer_;
};

// Implementation

template <typename TNextLayer>
template<typename... TParams>
LoggerDecoratorBase<TNextLayer>::LoggerDecoratorBase(TParams&&... params)
    : nextLayer_(std::forward<TParams>(params)...)
{
}

template <typename TNextLayer>
inline
typename LoggerDecoratorBase<TNextLayer>::Stream&
LoggerDecoratorBase<TNextLayer>::stream()
{
    return nextLayer_.stream();
}

template <typename TNextLayer>
inline
void LoggerDecoratorBase<TNextLayer>::begin(
    Level level)
{
    nextLayer_.begin(level);
}

template <typename TNextLayer>
inline
void LoggerDecoratorBase<TNextLayer>::end(
    Level level)
{
    nextLayer_.end(level);
}

}  // namespace details

}  // namespace log

}  // namespace util

}  // namespace embxx
