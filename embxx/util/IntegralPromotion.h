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

/// @file embxx/util/IntegralPromotion.h
/// This file contains classes that provide promotion to integral values.

#pragma once

#include <type_traits>

#include "embxx/util/SizeToType.h"

namespace embxx
{

namespace util
{

/// @ingroup util
/// @brief Implements integral promotion of the type preserving its "signed"
///        qualification.
/// @details This class doesn't preserve cv-qualifiers. int64_t and uint64_t
///          types are promoted to themselves.
/// @pre @code std::is_integral<T>::value is true. @endcode
/// @post @code std::is_signed<T>::value == std::is_signed<util::IntegralPromotion<T>::type>::value @endcode
/// @headerfile embxx/util/IntegralPromotion.h
template <typename T>
class IntegralPromotion
{
private:
    typedef typename std::decay<T>::type DecayType;
    static_assert(std::is_integral<T>::value, "T must be integral value");
public:

    /// Definition of the promoted type
    typedef typename SizeToType<
        sizeof(DecayType) + 1, std::is_signed<DecayType>::value
    >::Type Type;
};

/// @cond DOCUMENT_INTEGRAL_PROMOTION_SPECIALISATION
template <>
struct IntegralPromotion<std::int64_t>
{
    typedef std::int64_t Type;
};

template <>
struct IntegralPromotion<std::uint64_t>
{
    typedef std::uint64_t Type;
};
/// @endcond

}  // namespace util

}  // namespace embxx
