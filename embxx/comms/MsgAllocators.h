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

/// @file embxx/comms/MsgAllocators.h
/// This file contains definitions of various allocators for "comms" module.

#pragma once

#include "embxx/util/Allocators.h"

namespace embxx
{

namespace comms
{

/// @brief Message object allocation policy that uses dynamic memory allocation.
/// @details The newly allocated message object is returned wrapped in
///          standard std::unique_ptr with default deleter.
/// @headerfile embxx/comms/MsgAllocators.h
class DynMemMsgAllocator : public embxx::util::DynMemAllocator
{
};

/// @brief Message object allocation policy that uses "in place" object
///        construction.
/// @details It receives all the types of messages it can allocate wrapped in
///          std::tuple in single template parameter, calculates the required
///          size and alignment to be able to safely allocated any of the
///          required types and creates internal allocation space. The newly
///          constructed object is wrapped in std::unique_ptr with custom
///          deleter that calls destructor of the object.
/// @tparam TTuple std::tuple<...> with all the types of messages this allocator
///          can allocate
/// @headerfile embxx/comms/MsgAllocators.h
template <typename TAllMessages>
class InPlaceMsgAllocator : public embxx::util::SpecificInPlaceAllocator<TAllMessages>
{
};

}  // namespace comms

}  // namespace embxx
