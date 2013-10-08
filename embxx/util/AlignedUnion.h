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

/// @file embxx/util/AlignedUnion.h
/// This file contains basic implementation of std::aligned_union which
/// is not implemented in gcc 4.8.

#pragma once

#include <type_traits>

namespace embxx
{

namespace util
{

/// @brief Aligned union.
/// @details Defines the type suitable for use as uninitialised storage for
///          all given types
/// @tparam TType First type.
/// @tparam TTypes Zero or more other types
/// @headerfile embxx/util/AlignedUnion.h
template <typename TType, typename... TTypes>
class AlignedUnion
{
    typedef typename AlignedUnion<TTypes...>::Type OtherStorage;
    static const std::size_t OtherSize = sizeof(OtherStorage);
    static const std::size_t OtherAlignment = std::alignment_of<OtherStorage>::value;
    typedef typename AlignedUnion<TType>::Type FirstStorage;
    static const std::size_t FirstSize = sizeof(FirstStorage);
    static const std::size_t FirstAlignment = std::alignment_of<FirstStorage>::value;
    static const std::size_t MaxSize = FirstSize > OtherSize ? FirstSize : OtherSize;
    static const std::size_t MaxAlignment = FirstAlignment > OtherAlignment ? FirstAlignment : OtherAlignment;
public:
    /// Type that has proper size and proper alignment to keep any of the
    /// specified types
    typedef typename std::aligned_storage<MaxSize, MaxAlignment>::type Type;
};

/// @cond DOCUMENT_ALIGNED_UNION_SPECIALISATION

template <typename TType>
class AlignedUnion<TType>
{
public:
    typedef typename std::aligned_storage<sizeof(TType), std::alignment_of<TType>::value>::type Type;
};

/// @endcond


}  // namespace util

}  // namespace embxx
