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

/// @file embxx/io/traits.h
/// This file contains all the classes necessary to properly
/// define I/O traits.

#pragma once

namespace embxx
{

namespace io
{

namespace traits
{

namespace endian
{

/// @ingroup io
/// @brief Empty class used in traits to indicate Big Endian.
/// @headerfile embxx/io/traits.h
struct Big {};

/// @ingroup io
/// @brief Empty class used in traits to indicate Little Endian.
/// @headerfile embxx/io/traits.h
struct Little {};

}  // namespace endian

}  // namespace traits

}  // namespace io

}  // namespace embxx
