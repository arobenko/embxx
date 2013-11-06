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

namespace io
{

/// @addtogroup io
/// @{

/// @brief Type of embxx::io::endl stream manipulator;
enum Endl
{
    endl ///< End of line stream manipulator
};

/// @brief Type of embxx::io::ends stream manipulator;
enum Ends
{
    ends ///< End of string stream manipulator
};

/// @brief Type of numeric base stream manipulator;
enum Base
{
    bin, ///< Binary numeric base stream manipulator
    oct, ///< Octal numeric base stream manipulator
    dec, ///< Decimal numeric base stream manipulator
    hex, ///< Hexadecimal numeric base stream manipulator
    Base_NumOfBases ///< Must be last
};

/// @}


}  // namespace io

}  // namespace embxx


