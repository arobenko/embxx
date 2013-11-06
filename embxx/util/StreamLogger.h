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

/// @addtogroup util
/// @{

/// @brief Base class for Stream Logger
/// @details The logger object just wraps reference to output stream object
///          and returns it upon request.
/// @tparam TStream Type of the output stream object. May be either std::ostream
///         or embxx::io::OutStream
/// @headerfile embxx/util/StreamLogger.h
template <typename TStream>
class StreamLoggerBase
{
public:

    /// @brief Type of the output stream.
    typedef TStream Stream;

    /// @brief Constructor
    /// @param[in] outStream referenct to output stream
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    inline StreamLoggerBase(Stream& outStream);

    /// @brief Ger reference to output stream
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    inline Stream& stream();

    /// @brief Begin output.
    /// @details This function is called before requested output is redirected
    ///          to stream. It does nothing.
    /// @param[in] level Logging level
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    inline void begin(log::Level level);

    /// @brief End output.
    /// @details This function is called after requested output is redirected
    ///          to stream. It does nothing.
    /// @param[in] level Logging level
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    inline void end(log::Level level);

private:
    Stream& outStream_;
};

/// @brief Stream Logger class
/// @brief The only thing is adds to its base class (StreamLoggerBase) is
///        the compile time information of minimal log level.
/// @headerfile embxx/util/StreamLogger.h
template <log::Level TLevel, typename TStream>
class StreamLogger : public StreamLoggerBase<TStream>
{
    typedef StreamLoggerBase<TStream> Base;
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
    inline StreamLogger(Stream& outStream);
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

// Implementation area

template <typename TStream>
inline
StreamLoggerBase<TStream>::StreamLoggerBase(Stream& outStream)
    : outStream_(outStream)
{
}

template <typename TStream>
inline
typename StreamLoggerBase<TStream>::Stream&
StreamLoggerBase<TStream>::stream()
{
    return outStream_;
}

template <typename TStream>
inline
void StreamLoggerBase<TStream>::begin(log::Level level)
{
    // Do nothing
    static_cast<void>(level);
}

template <typename TStream>
inline
void StreamLoggerBase<TStream>::end(log::Level level)
{
    // Do nothing
    static_cast<void>(level);
}

template <log::Level TLevel, typename TStream>
inline
StreamLogger<TLevel, TStream>::StreamLogger(Stream& outStream)
    : Base(outStream)
{
}

}  // namespace util

}  // namespace embxx
