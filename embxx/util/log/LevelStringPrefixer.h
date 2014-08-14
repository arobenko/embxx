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

/// @file embxx/util/log/LevelStringPrefixer.h
/// Contains definition of LevelStringPrefixer.

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

/// @brief Defines decorator that prints log level before the requested
///        output
/// @details The output will be one of the following strings:
///          @li [DEBUG]
///          @li [TRACE]
///          @li [INFO]
///          @li [WARNING]
///          @li [ERROR]
/// @tparam TNextLayer Next layer type
/// @headerfile embxx/util/log/LevelStringPrefixer.h
template <typename TNextLayer>
class LevelStringPrefixer : public details::LoggerDecoratorBase<TNextLayer>
{
    typedef details::LoggerDecoratorBase<TNextLayer> Base;
public:

    /// @brief Constructor
    /// @details Forwards all the parameters to the constructor of the next layer
    /// @param[in] params Zero or more parameters to be forwarded to the next layer.
    /// @note Exception guarantee: Strong
    template<typename... TParams>
    explicit LevelStringPrefixer(TParams&&... params)
      : Base(std::forward<TParams>(params)...)
    {
    }


    /// @brief Prints log level in square brackets prior
    ///        to redirecting the request to the next layer.
    /// @param[in] level requested logging level
    void begin(Level level)
    {
        static const char* const Strings[NumOfLogLevels] = {
            "[TRACE] ",
            "[DEBUG] ",
            "[INFO] ",
            "[WARNING] ",
            "[ERROR] "
        };

        if ((level < NumOfLogLevels) && (Strings[level] != nullptr)) {
            Base::stream() << Strings[level];
        }

        Base::begin(level);
    }

    /// @brief Calls "end()" member function of the next layer.
    /// @details No additional formatting.
    /// @param[in] level requested logging level
    void end(Level level)
    {
        Base::end(level);
    }
};

/// @}

}  // namespace log

}  // namespace util

}  // namespace embxx
