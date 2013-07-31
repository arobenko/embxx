//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file io/traits.h
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
struct Big {};

/// @ingroup io
/// @brief Empty class used in traits to indicate Little Endian.
struct Little {};

}  // namespace endian

}  // namespace traits

}  // namespace io

}  // namespace embxx
