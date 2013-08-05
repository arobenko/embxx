//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file util/IntegralPromotion.h
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
