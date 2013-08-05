//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file comms/traits.h
/// This file contains all the classes necessary to properly
/// define message traits.


#pragma once

#include "embxx/io/traits.h"

namespace embxx
{

namespace comms
{

namespace traits
{

/// @ingroup comms
/// @brief Type used for message IDs
/// @headerfile comms/traits.h "comms/traits.h"
typedef unsigned MsgIdType;

namespace endian
{

/// @ingroup comms
/// @brief Empty class used in traits to indicate Big Endian.
/// @headerfile comms/traits.h "comms/traits.h"
typedef io::traits::endian::Big Big;

/// @ingroup comms
/// @brief Empty class used in traits to indicate Little Endian.
/// @headerfile comms/traits.h "comms/traits.h"
typedef io::traits::endian::Little Little;

}  // namespace endian

namespace checksum
{

/// @ingroup comms
/// @brief Empty class in traits to indicate that checksum verification
///        must be done before passing data to the next protocol layer
/// @headerfile comms/traits.h "comms/traits.h"
struct VerifyBeforeProcessing {};

/// @ingroup comms
/// @brief Empty class in traits to indicate that checksum verification
///        must be done after passing data to the next protocol layer
///        and successful processing of the latter.
/// @headerfile comms/traits.h "comms/traits.h"
struct VerifyAfterProcessing {};

} // namespace checksum

}  // namespace traits

}  // namespace comms

}  // namespace embxx
