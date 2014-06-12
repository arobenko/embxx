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

/// @file embxx/comms/field/BasicIntValue.h
/// This file contains definition of basic integral value field that
/// can be used in message definition.

#pragma once

#include <type_traits>

#include "embxx/util/Assert.h"
#include "embxx/util/SizeToType.h"
#include "embxx/util/IntegralPromotion.h"
#include "embxx/io/access.h"
#include "embxx/comms/ErrorStatus.h"

namespace embxx
{

namespace comms
{

namespace field
{

/// @addtogroup comms
/// @{

/// @brief Defines "Basic Integral Value Field".
/// @details The class provides an API to access the value of the field as
///          well as to serialise/deserialise the value to/from stream buffer.
/// @tparam T Integral value type.
/// @tparam TTraits Various behavioural traits relevant for the field.
///         Currently the only trait that is required for this class is
///         Endianness. The traits class/struct must typedef either
///         embxx::comms::traits::endian::Big or
///         embxx::comms::traits::endian::Little to Endianness.
/// @tparam TLen Length of serialised data in bytes. Default value is sizeof(T).
/// @tparam TOff Offset to apply when serialising data.
/// @pre @code std::is_integral<T>::value == true @endcode
/// @headerfile embxx/comms/field/BasicIntValue.h
template <typename T,
          typename TTraits,
          std::size_t TLen = sizeof(T),
          typename util::IntegralPromotion<T>::Type TOff = static_cast<typename util::IntegralPromotion<T>::Type>(0) >
class BasicIntValue
{
    static_assert(std::is_integral<T>::value, "T must be integral value");

public:

    /// @brief Value Type
    typedef T ValueType;

    /// @brief Serialised Type
    typedef typename util::SizeToType<TLen, std::is_signed<T>::value>::Type SerialisedType;

    /// @brief Field traits
    typedef TTraits Traits;

    /// @brief Data endianness
    typedef typename Traits::Endianness Endianness;

    /// @brief Length of serialised data
    static const std::size_t SerialisedLen = TLen;

    /// @brief Offset to be applied when serialising
    static const typename util::IntegralPromotion<T>::Type Offset = TOff;

    /// @brief Default constructor
    /// @details Sets default value to be 0.
    BasicIntValue();

    /// @brief Constructor
    /// @details Sets initial value.
    /// @param value Initial value
    explicit BasicIntValue(ValueType value);

    /// @brief Copy constructor is default
    BasicIntValue(const BasicIntValue&) = default;

    /// @brief Destructor is default
    ~BasicIntValue() = default;

    /// @brief Copy assignment is default
    BasicIntValue& operator=(const BasicIntValue&) = default;

    /// @brief Retrieve the value.
    const ValueType getValue() const;

    /// @brief Set the value
    /// @param value Value to set.
    void setValue(ValueType value);

    /// @brief Retrieve serialised data
    const SerialisedType getSerialisedValue() const;

    /// @brief Set serialised data
    void setSerialisedValue(SerialisedType value);

    /// @brief Convert value to serialised data
    static constexpr const SerialisedType toSerialised(ValueType value);

    /// @brief Convert serialised data to actual value
    static constexpr const ValueType fromSerialised(SerialisedType value);

    /// @brief Get length of serialised data
    static constexpr std::size_t length();

    /// @brief Read the serialised field value from the some data structure.
    /// @tparam TIter Type of input iterator
    /// @param[in, out] iter Input iterator.
    /// @param[in] size Size of the data in iterated data structure.
    /// @return Status of the read operation.
    /// @pre Value of provided "size" must be less than or equal to
    ///      available data in the used data structure/stream
    /// @post The iterator will be incremented.
    template <typename TIter>
    ErrorStatus read(TIter& iter, std::size_t size);

    /// @brief Write the serialised field value to some data structure.
    /// @tparam TIter Type of output iterator
    /// @param[in, out] iter Output iterator.
    /// @param[in] size Size of the buffer, field data must fit it.
    /// @return Status of the write operation.
    /// @pre Value of provided "size" must be less than or equal to
    ///      available space in the data structure.
    /// @post The iterator will be incremented.
    template <typename TIter>
    ErrorStatus write(TIter& iter, std::size_t size) const;

private:
    ValueType value_;
};


// Implementation

/// @brief Equality comparison operator.
/// @related BasicIntValue
template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
bool operator==(
    const BasicIntValue<T, TTraits, TLen, TOff>& field1,
    const BasicIntValue<T, TTraits, TLen, TOff>& field2)
{
    return field1.getValue() == field2.getValue();
}

/// @brief Non-equality comparison operator.
/// @related BasicIntValue
template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
bool operator!=(
    const BasicIntValue<T, TTraits, TLen, TOff>& field1,
    const BasicIntValue<T, TTraits, TLen, TOff>& field2)
{
    return field1.getValue() != field2.getValue();
}


/// @brief Equivalence comparison operator.
/// @related BasicIntValue
template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
bool operator<(
    const BasicIntValue<T, TTraits, TLen, TOff>& field1,
    const BasicIntValue<T, TTraits, TLen, TOff>& field2)
{
    return field1.getValue() < field2.getValue();
}

/// @}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
BasicIntValue<T, TTraits, TLen, TOff>::BasicIntValue()
    : value_(static_cast<ValueType>(0))
{
}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
BasicIntValue<T, TTraits, TLen, TOff>::BasicIntValue(ValueType value)
    : value_(value)
{
}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
const typename BasicIntValue<T, TTraits, TLen, TOff>::ValueType
BasicIntValue<T, TTraits, TLen, TOff>::getValue() const
{
    return value_;
}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
void BasicIntValue<T, TTraits, TLen, TOff>::setValue(ValueType value)
{
    value_ = value;
}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
const typename BasicIntValue<T, TTraits, TLen, TOff>::SerialisedType
BasicIntValue<T, TTraits, TLen, TOff>::getSerialisedValue() const
{
    return toSerialised(value_);
}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
void BasicIntValue<T, TTraits, TLen, TOff>::setSerialisedValue(SerialisedType value)
{
    value_ = fromSerialised(value);
}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
constexpr
const typename BasicIntValue<T, TTraits, TLen, TOff>::SerialisedType
BasicIntValue<T, TTraits, TLen, TOff>::toSerialised(ValueType value)
{
    return static_cast<SerialisedType>(Offset + value);
}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
constexpr
const typename BasicIntValue<T, TTraits, TLen, TOff>::ValueType
BasicIntValue<T, TTraits, TLen, TOff>::fromSerialised(SerialisedType value)
{
    return static_cast<ValueType>((-Offset) + value);
}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
constexpr std::size_t BasicIntValue<T, TTraits, TLen, TOff>::length()
{
    return SerialisedLen;
}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
template <typename TIter>
ErrorStatus BasicIntValue<T, TTraits, TLen, TOff>::read(
    TIter& iter,
    std::size_t size)
{
    if (size < length()) {
        return ErrorStatus::NotEnoughData;
    }

    auto serialisedValue =
        io::readData<SerialisedType, SerialisedLen>(
            iter,
            Endianness());
    setSerialisedValue(serialisedValue);
    return ErrorStatus::Success;
}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
template <typename TIter>
ErrorStatus BasicIntValue<T, TTraits, TLen, TOff>::write(
    TIter& iter,
    std::size_t size) const
{
    GASSERT(length() <= size);
    static_cast<void>(size);

    io::writeData<SerialisedLen>(getSerialisedValue(), iter, Endianness());
    return ErrorStatus::Success;
}


}  // namespace field

}  // namespace comms

}  // namespace embxx
