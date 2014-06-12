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

namespace device
{

namespace context
{

/// @ingroup device
/// @brief Event loop context tag class.
/// @details This is empty struct that is used to indicate event loop execution context.
/// @headerfile embxx/device/context.h
struct EventLoop {};

/// @ingroup device
/// @brief Interrupt context tag class.
/// @details This is empty struct that is used to indicate interrupt execution context.
/// @headerfile embxx/device/context.h
struct Interrupt {};

}  // namespace context

}  // namespace device

}  // namespace embxx


