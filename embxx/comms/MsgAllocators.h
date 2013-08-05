//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

#pragma once

#include "embxx/util/Allocators.h"

namespace embxx
{

namespace comms
{

/// @brief Message object allocation policy that uses dynamic memory allocation.
/// @details The newly allocated message object is returned wrapped in
///          standard std::unique_ptr with default deleter.
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
template <typename TAllMessages>
class InPlaceMsgAllocator : public embxx::util::SpecificInPlaceAllocator<TAllMessages>
{
};

}  // namespace comms

}  // namespace embxx
