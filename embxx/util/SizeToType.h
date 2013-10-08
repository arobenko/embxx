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

/// @file embxx/util/SizeToType.h
/// Contains specialisation of SizeToType class for several sizes.

#pragma once

#include <array>
#include <cstdint>

namespace embxx
{

namespace util
{

namespace details
{

template <std::size_t TSize>
struct SizeToTypeHelper {
    typedef std::array<std::uint8_t, TSize> Type;
};

template <>
struct SizeToTypeHelper<1>
{
    typedef std::uint8_t Type;
};

template <>
struct SizeToTypeHelper<2>
{
    typedef std::uint16_t Type;
};

template <>
struct SizeToTypeHelper<4>
{
    typedef std::uint32_t Type;
};

template <>
struct SizeToTypeHelper<8>
{
    typedef std::uint64_t Type;
};

template <>
struct SizeToTypeHelper<3>
{
    typedef typename SizeToTypeHelper<4>::Type Type;
};

template <>
struct SizeToTypeHelper<5>
{
    typedef typename SizeToTypeHelper<8>::Type Type;
};

template <>
struct SizeToTypeHelper<6>
{
    typedef typename SizeToTypeHelper<8>::Type Type;
};

template <>
struct SizeToTypeHelper<7>
{
    typedef typename SizeToTypeHelper<8>::Type Type;
};


}  // namespace details

/// @addtogroup util
/// @{

/// @brief Type information based on size known at compile time.
/// @details Maps size to unsigned integral type. If signed type is
///          needed, set TSigned teplate parameter to true.
///          The types are as following:
///          @li SizeToType<1>::Type is std::uint8_t
///          @li SizeToType<1, true>::Type is std::int8_t
///          @li SizeToType<2>::Type is std::uint16_t
///          @li SizeToType<2, true>::Type is std::int16_t
///          @li SizeToType<3>::Type is std::uint32_t
///          @li SizeToType<3, true>::Type is std::int32_t
///          @li SizeToType<4>::Type is std::uint32_t
///          @li SizeToType<4, true>::Type is std::int32_t
///          @li SizeToType<5>::Type is std::uint64_t
///          @li SizeToType<5, true>::Type is std::int64_t
///          @li SizeToType<6>::Type is std::uint64_t
///          @li SizeToType<6, true>::Type is std::int64_t
///          @li SizeToType<7>::Type is std::uint64_t
///          @li SizeToType<7, true>::Type is std::int64_t
///          @li SizeToType<8>::Type is std::uint64_t
///          @li SizeToType<8, true>::Type is std::int64_t
///          @li All other sizes are typedefed to std::array<std::uint8_t, Size>
///              or std::array<std::int8_t, Size> based on TSigned parameter
/// @tparam TSize Size, compile time constant
/// @tparam TSigned Specifies whether the type must be signed. False by default
/// @headerfile embxx/util/SizeToType.h
template <std::size_t TSize, bool TSigned = false>
class SizeToType
{
    typedef typename SizeToType<1, TSigned>::Type ByteType;

public:
    /// Generic type definition
    typedef std::array<ByteType, TSize> Type;
};

/// @cond DOCUCMENT_SIZE_TO_TYPE_SPECIALISATION
template <std::size_t TSize>
struct SizeToType<TSize, false>
{
    typedef typename details::SizeToTypeHelper<TSize>::Type Type;
};

template <std::size_t TSize>
struct SizeToType<TSize, true>
{
    typedef typename
        std::make_signed<
            typename SizeToType<TSize, false>::Type
        >::type Type;
};

/// @endcond

/// @}

}  // namespace util

}  // namespace embxx
