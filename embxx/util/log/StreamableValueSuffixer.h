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

/// @file embxx/util/log/StreamableValueSuffixer.h
/// Contains definition of StreamableValueSuffixer decorator of the StreamLogger.

#pragma once

#include "StreamableValueDecoratorBase.h"

namespace embxx
{

namespace util
{

namespace log
{

/// @addtogroup util
/// @{

/// @brief Defines a decorator that redirects provided value to the output
///        stream afther the call to end() member function of the next layer.
/// @headerfile embxx/util/log/StreamableValueSuffixer.h
template <typename T, typename TNextLayer>
class StreamableValueSuffixer : public details::StreamableValueDecoratorBase<T, TNextLayer>
{
    typedef details::StreamableValueDecoratorBase<T, TNextLayer> Base;
public:

    /// Redefinition of the Value Type
    typedef typename Base::ValueType ValueType;

    /// @brief Constructor
    /// @details Keeps a copy of the provided streamable value and forwards all
    ///          the other parameters to the constructor of the next layer.
    /// @param[in] value streamable value
    /// @param[in] params Zero or more parameters to be forwarded to the next layer.
    /// @note Exception guarantee: Strong
    template<typename... TParams>
    explicit StreamableValueSuffixer(ValueType&& value, TParams&&... params)
      : Base(std::forward<ValueType>(value), std::forward<TParams>(params)...)
    {
    }

    /// @brief Calls "begin()" member function of the next layer.
    /// @details No additional formatting.
    /// @param[in] level requested logging level
    void begin(Level level)
    {
        Base::begin(level);
    }

    /// @brief Redirects provided streamable value to the output stream after
    ///        call to the end() member function of the next layer.
    /// @param[in] level requested logging level
    /// @note Exception guarantee: Strong
    void end(Level level)
    {
        Base::end(level);
        Base::stream() << Base::value_;
    }
};

/// @}

}  // namespace log

}  // namespace util

}  // namespace embxx
