//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file util/StreamLogger.h
/// This file provides main definition of StreamLogger class.

#pragma once

#include <iosfwd>

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
/// @headerfile embxx/util/StreamLogger.h
class StreamLoggerBase
{
public:
    /// @brief Constructor
    /// @param[in] outStream referenct to output stream
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    inline StreamLoggerBase(std::ostream& outStream);

    /// @brief Ger reference to output stream
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    inline std::ostream& stream();

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
    std::ostream& outStream_;
};

/// @brief Stream Logger class
/// @brief The only thing is adds to its base class (StreamLoggerBase) is
///        the compile time information of minimal log level.
/// @headerfile embxx/util/StreamLogger.h
template <log::Level TLevel>
class StreamLogger : public StreamLoggerBase
{
    typedef StreamLoggerBase Base;
public:
    /// Minimal log level. The output will be redirected to the stream
    /// if and only if requested level is greater or equal to MinLevel.
    static const log::Level MinLevel = TLevel;

    /// @brief Constructor
    /// @param[in] outStream Reference to output stream
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    inline StreamLogger(std::ostream& outStream);
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
inline
StreamLoggerBase::StreamLoggerBase(std::ostream& outStream)
    : outStream_(outStream)
{
}

inline
std::ostream& StreamLoggerBase::stream()
{
    return outStream_;
}

inline
void StreamLoggerBase::begin(log::Level level)
{
    // Do nothing
    static_cast<void>(level);
}

inline
void StreamLoggerBase::end(log::Level level)
{
    // Do nothing
    static_cast<void>(level);
}

template <log::Level TLevel>
inline
StreamLogger<TLevel>::StreamLogger(std::ostream& outStream)
    : Base(outStream)
{
}

}  // namespace util

}  // namespace embxx
