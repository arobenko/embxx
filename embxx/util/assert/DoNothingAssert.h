//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

// This file is free software: you can redistribute it and/or modify
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


#pragma once

namespace embxx
{

namespace util
{

namespace assert
{
/// @addtogroup util
///
/// @{

/// @brief Do nothing assert.
/// @details Causes the assertion failure to be ignored.
/// @headerfile embxx/util/assert/DoNothingAssert.h
class DoNothingAssert : public embxx::util::Assert
{
public:
    /// @brief Overrides fail() member function of base class
    /// @details Does nothing.
    /// @param[in] expr Assertion condition/expression
    /// @param[in] file File name
    /// @param[in] line Line number of the assert statement.
    /// @param[in] function Function name.
    /// @note Thread safety: Safe.
    /// @note Exception guarantee: No throw.
    virtual void fail(
        const char* expr,
        const char* file,
        unsigned int line,
        const char* function) override
    {
        static_cast<void>(expr);
        static_cast<void>(file);
        static_cast<void>(line);
        static_cast<void>(function);
    }
};

/// @}

}  // namespace assert

}  // namespace util

}  // namespace embxx
