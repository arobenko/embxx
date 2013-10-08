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

/// @file embxx/util/log/LogLevel.h
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
