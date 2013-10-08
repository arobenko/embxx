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

/// @file embxx/util/Tuple.h
/// This file contains various utilities that work with std::tuple.

#pragma once

#include <tuple>
#include <type_traits>

#include "embxx/util/AlignedUnion.h"

namespace embxx
{

namespace util
{

/// @addtogroup util
/// @{

/// @brief Compile time check whether a type of a variant of std::tuple.
/// @details Static const data member Value will be true in case TType
///          is any variant of std::tuple, false otherwise.
/// @tparam TType Any type
/// @headerfile embxx/util/Tuple.h
template <typename TType>
struct IsTuple
{
    /// @brief By default Value has value false. Will be true for any
    /// variant of std::tuple.
    static const bool Value = false;
};

/// @cond DOCUMENT_IS_TUPLE_SPECIALISATION

template <typename... TArgs>
struct IsTuple<std::tuple<TArgs...> >
{
    static const bool Value = true;
};

/// @endcond

//----------------------------------------

/// @brief Compile time check whether provided type is in provided tuple.
/// @details Static const data member "Value" will be true if TType is in
///          tuple TTuple
/// @tparam TType Any type
/// @tparam TTuple Must be any kind of std::tuple
/// @pre TTuple is any kind of std::tuple
/// @headerfile embxx/util/Tuple.h
template <typename TType, typename TTuple>
class IsInTuple
{
    static_assert(IsTuple<TTuple>::Value, "TTuple must be std::tuple");
public:
    /// @brief By default Value will be false. Will be true if TType is in
    /// TTuple.
    static const bool Value = false;
};

/// @cond DOCUMENT_IS_IN_TUPLE_SPECIALISATION
template <typename TType, typename TFirst, typename... TRest>
class IsInTuple<TType, std::tuple<TFirst, TRest...> >
{
public:
    static const bool Value =
        std::is_same<TType, TFirst>::value ||
        IsInTuple<TType, std::tuple<TRest...> >::Value;
};

template <typename TType>
class IsInTuple<TType, std::tuple<> >
{
public:
    static const bool Value = false;
};

/// @endcond

//----------------------------------------

/// @brief Provides uninitialised storage type that satisfies
///        size and alignment requirements to contain any type from
///        std::tuple
/// @details Internal "Type" type definition will be the type of proper
///          storage area.
/// @headerfile embxx/util/Tuple.h
template <typename TTuple>
struct TupleAsAlignedUnion
{
    /// @cond DOCUMENT_STATIC_ASSERT
    static_assert(IsTuple<TTuple>::Value, "TTuple must be std::tuple");
    /// @endcond

    /// @brief Type definition is invalid for any type that is not std::tuple.
    typedef void Type;
};

/// @cond DOCUMENT_TUPLE_AS_ALIGNED_UNION_SPECIALISATION

template <typename... TTypes>
struct TupleAsAlignedUnion<std::tuple<TTypes...> >
{
    typedef typename AlignedUnion<TTypes...>::Type Type;
};

/// @endcond

//----------------------------------------

/// @brief Check whether std::tuple is unique, i.e. all types inside have only
///        one occurrence of each.
/// @headerfile embxx/util/Tuple.h
template <typename TTuple>
struct TupleIsUnique
{
    /// @cond DOCUMENT_STATIC_ASSERT
    static_assert(IsTuple<TTuple>::Value, "TTuple must be std::tuple");
    /// @endcond

    /// @brief Value will be true if std::tuple is inique, false otherwise.
    static const bool Value = false;
};

/// @cond DOCUMENT_TUPLE_IS_UNIQUE_SPECIALISATION
template <typename TFirst, typename... TRest>
struct TupleIsUnique<std::tuple<TFirst, TRest...> >
{
    static const bool Value =
        (!IsInTuple<TFirst, std::tuple<TRest...> >::Value) &&
        TupleIsUnique<std::tuple<TRest...> >::Value;
};

template <>
struct TupleIsUnique<std::tuple<> >
{
    static const bool Value = true;
};
/// @endcond

//----------------------------------------

/// @cond DOCUMENT_TUPLE_FOR_EACH_HELPER
template <std::size_t TIdx>
class TupleForEachHelper
{

public:
    template <typename TTuple, typename TFunc>
    static void exec(TTuple&& tuple, TFunc&& func)
    {
        typedef typename std::decay<TTuple>::type Tuple;
        static_assert(IsTuple<Tuple>::Value, "TTuple must be std::tuple");
        static const std::size_t TupleSize = std::tuple_size<Tuple>::value;
        static_assert(TIdx <= TupleSize, "Incorrect TIdx");

        static const std::size_t Idx = TupleSize - TIdx;
        func(std::get<Idx>(std::forward<TTuple>(tuple)));
        TupleForEachHelper<TIdx - 1>::exec(
            std::forward<TTuple>(tuple),
            std::forward<TFunc>(func));
    }
};

template <>
class TupleForEachHelper<0>
{

public:
    template <typename TTuple, typename TFunc>
    static void exec(TTuple&& tuple, TFunc&& func)
    {
        static_cast<void>(tuple);
        static_cast<void>(func);
    }
};
/// @endcond

/// @brief Iterate over all elements of tuple and execute provided functor.
/// @details Requires functor to have operator() with one of the following
///          signatures depending on const specification of the provided tuple.
///          @li @code template <typename T> void operator()(T& element); @endcode
///          @li @code template <typename T> void operator()(const T& element); @endcode
/// @param[in] tuple Tuple
/// @param[in] func Functor
/// @pre tuple is any variant of std::tuple
template <typename TTuple, typename TFunc>
void tupleForEach(TTuple&& tuple, TFunc&& func)
{
    typedef typename std::decay<TTuple>::type Tuple;
    static const std::size_t TupleSize = std::tuple_size<Tuple>::value;

    TupleForEachHelper<TupleSize>::exec(
        std::forward<TTuple>(tuple),
        std::forward<TFunc>(func));
}
//----------------------------------------

/// @cond DOCUMENT_TUPLE_ACCUMULATE_HELPER
template <std::size_t TIdx>
class TupleAccumulateHelper
{

public:
    template <typename TTuple, typename TValue, typename TFunc>
    static TValue exec(TTuple&& tuple, const TValue& value, TFunc&& func)
    {
        typedef typename std::decay<TTuple>::type Tuple;
        static_assert(IsTuple<Tuple>::Value, "TTuple must be std::tuple");
        static const std::size_t TupleSize = std::tuple_size<Tuple>::value;
        static_assert(TIdx <= TupleSize, "Incorrect TIdx");

        static const std::size_t Idx = TupleSize - TIdx;
        return TupleAccumulateHelper<TIdx - 1>::exec(
                    std::forward<TTuple>(tuple),
                    func(value, std::get<Idx>(std::forward<TTuple>(tuple))),
                    std::forward<TFunc>(func));
    }
};

template <>
class TupleAccumulateHelper<0>
{

public:
    template <typename TTuple, typename TValue, typename TFunc>
    static TValue exec(TTuple&& tuple, const TValue& value, TFunc&& func)
    {
        static_cast<void>(tuple);
        static_cast<void>(func);
        return value;
    }
};
/// @endcond

/// @brief Perform "accumulate" algorithm over all elements of the tuple.
/// @details Requires functor to have operator() with one of the following
///          signatures depending on const specification of the provided tuple.
///          @li @code template <typename TValue, typename T> void operator()(const TValue& value, T& element); @endcode
///          @li @code template <typename TValue, typename T> void operator()(const TValue& value, const T& element); @endcode
/// @return The result of the repeated application of binary func to the result of the previous func invocation (inital_state if it is the first call) and each element of tuple
/// @param[in] tuple Tuple
/// @param[in] value Initial value.
/// @param[in] func Binary function
/// @pre tuple is any variant of std::tuple
/// @pre func is a binary function first argument of which must have
///      const TValue& type.
template <typename TTuple, typename TValue, typename TFunc>
TValue tupleAccumulate(TTuple&& tuple, const TValue& value, TFunc&& func)
{
    typedef typename std::decay<TTuple>::type Tuple;
    static const std::size_t TupleSize = std::tuple_size<Tuple>::value;

    return TupleAccumulateHelper<TupleSize>::exec(
                std::forward<TTuple>(tuple),
                value,
                std::forward<TFunc>(func));
}

/// @}

}  // namespace util

}  // namespace embxx
