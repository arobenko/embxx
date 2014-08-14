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

/// @file embxx/util/log/StreamFlushSuffixer.h
/// Contains definition of StreamFlushSuffixer decorator of the StreamLogger.

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

/// @brief Defines a decorator that calls flush() method of the
///        provided stream after the call to the end() member function
///        of the next layer.
/// @headerfile embxx/util/log/StreamFlushSuffixer.h
template <typename TNextLayer>
class StreamFlushSuffixer : public details::LoggerDecoratorBase<TNextLayer>
{
    typedef details::LoggerDecoratorBase<TNextLayer> Base;
public:

    /// @brief Constructor
    /// @details Forwards all the other parameters to the constructor of
    ///          the next layer.
    /// @param[in] params Zero or more parameters to be forwarded to the next layer.
    /// @note Exception guarantee: Strong
    template<typename... TParams>
    explicit StreamFlushSuffixer(TParams&&... params)
      : Base(std::forward<TParams>(params)...)
    {
    }

    /// @brief Calls "begin()" member function of the next layer.
    /// @details No additional formatting.
    /// @param[in] level requested logging level
    void begin(Level level)
    {
        Base::begin(level);
    }

    /// @brief Calls "flush()" method of the output stream after calling
    ///        end() member function of the next layer.
    /// @param[in] level requested logging level
    /// @note Exception guarantee: Strong
    void end(Level level)
    {
        Base::end(level);
        Base::stream().flush();
    }
};

/// @}

}  // namespace log

}  // namespace util

}  // namespace embxx

