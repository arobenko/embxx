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

/// @file embxx/util/StreamLogger.h
/// This file provides main definition of StreamLogger class.

#pragma once

#include "log/LogLevel.h"

namespace embxx
{

namespace util
{

namespace details
{

template <typename TStream>
class StreamLoggerBase
{
public:

    typedef TStream Stream;

    inline StreamLoggerBase(Stream& outStream)
      : outStream_(outStream)
    {
    }

    Stream& stream()
    {
        return outStream_;
    }

    void begin(log::Level level)
    {
        // Do nothing
        static_cast<void>(level);
    }

    void end(log::Level level)
    {
        // Do nothing
        static_cast<void>(level);
    }

private:
    Stream& outStream_;
};

}  // namespace details

/// @addtogroup util
/// @{

/// @brief Stream Logger
/// @details The logger object just wraps reference to output stream object
///          and returns it upon request.
/// @tparam TLevel Minimal logging level.
/// @tparam TStream Type of the output stream object. May be either std::ostream
///         or embxx::io::OutStream.
/// @headerfile embxx/util/StreamLogger.h
template <log::Level TLevel, typename TStream>
class StreamLogger : public details::StreamLoggerBase<TStream>
{
    typedef details::StreamLoggerBase<TStream> Base;
public:

    /// @brief Type of the output stream object.
    typedef typename Base::Stream Stream;

    /// @brief Minimal log level.
    /// @details The output will be redirected to the stream
    /// if and only if requested level is greater or equal to MinLevel.
    static const log::Level MinLevel = TLevel;

    /// @brief Constructor
    /// @param[in] outStream Reference to output stream
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    explicit StreamLogger(Stream& outStream)
      : Base(outStream)
    {
    }

    /// @brief Get reference to output stream
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    Stream& stream()
    {
        return Base::stream();
    }

    /// @brief Begin output.
    /// @details This function is called before requested output is redirected
    ///          to stream. It does nothing.
    /// @param[in] level Logging level
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    void begin(log::Level level)
    {
        Base::begin(level);
    }

    /// @brief End output.
    /// @details This function is called after requested output is redirected
    ///          to stream. It does nothing.
    /// @param[in] level Logging level
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    void end(log::Level level)
    {
        Base::end(level);
    }
};

/// @brief Logging macro
/// @param log__ reference to logger object
/// @param level__ requested logging level
/// @param output__ requested output
#define SLOG(log__, level__, output__) \
    do { \
        if ((log__).MinLevel <= (level__)) { \
            (log__).begin(level__); \
            (log__).stream() << output__; \
            (log__).end(level__); \
        } \
    } while (false)

/// @}

}  // namespace util

}  // namespace embxx
