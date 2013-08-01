//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file util/log/LogLevel.h
/// Contains definition of the predefined log levels.

#pragma once

namespace embxx
{

namespace util
{

namespace log
{

/// @addtogroup util
/// @{

/// @brief Logging levels
enum Level
{
    Trace, ///< Use for tracing enter to and exit from functions.
    Debug, ///< Use for debugging information.
    Info, ///< Use for general informative output.
    Warning, ///< Use for warning about potential dangers.
    Error, ///< Use to report execution errors.
    NumOfLogLevels ///< Number of log levels, must be last
};

/// @}

}  // namespace log

}  // namespace util

}  // namespace embxx
