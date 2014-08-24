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

/// @file embxx/util/log/StreamableValueDecoratorBase.h
/// Contains definition of StreamableValueDecoratorBase.

#pragma once

#include "LoggerDecoratorBase.h"

namespace embxx
{

namespace util
{

namespace log
{

namespace details
{

/// @brief Base class for the StreamableValuePrefixer and
///        StreamableValueSuffixer
/// @details It contains common functionality for both dirived classes
/// @tparam T Streamable type
/// @tparam TNextLayer Next layer type
/// @headerfile embxx/util/log/StreamableValueDecoratorBase.h
template <typename T, typename TNextLayer>
class StreamableValueDecoratorBase : public LoggerDecoratorBase<TNextLayer>
{
    typedef LoggerDecoratorBase<TNextLayer> Base;
public:
    /// Type of the streamable value
    typedef T ValueType;

    /// @brief Constructor
    /// @details Keeps a copy of the provided streamable value and forwards all
    ///          the other parameters to the constructor of the next layer.
    /// @param[in] value streamable value
    /// @param[in] params Zero or more parameters to be forwarded to the next layer.
    /// @note Exception guarantee: Strong
    template<typename... TParams>
    explicit StreamableValueDecoratorBase(ValueType&& value, TParams&&... params)
      : Base(std::forward<TParams>(params)...),
        value_(std::forward<ValueType>(value))
    {
    }

    StreamableValueDecoratorBase(const StreamableValueDecoratorBase&) = default;
    StreamableValueDecoratorBase& operator=(const StreamableValueDecoratorBase&) = default;

protected:
    ValueType value_; ///< Streamable value
};

} // namespace details

}  // namespace log

}  // namespace util

}  // namespace embxx
