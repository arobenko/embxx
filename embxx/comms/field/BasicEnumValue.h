//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

// This file is free software: you can redistribute it and/or modify
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


#pragma once

#include <cstdint>
#include <limits>
#include <type_traits>

#include "embxx/util/SizeToType.h"

#include "BasicIntValue.h"

namespace embxx
{

namespace comms
{

namespace field
{

/// @addtogroup comms
/// @{

/// @brief Defines "Enum Value Field".
/// @details The class uses BasicIntValue as its underlying type while providing
///          additional API function to check range of the value.
/// @tparam TEnum Enum type.
/// @tparam TTraits Various behavioural traits relevant for the field.
///         Currently the only trait that is required for this class is
///         Endianness. The traits class/struct must typedef either
///         embxx::comms::traits::endian::Big or
///         embxx::comms::traits::endian::Little to Endianness.
/// @tparam TLen Length of serialised data in bytes. The default value is
///         sizeof(std::uint8_t).
/// @tparam TLimit Maximal and invalid value. All other values below the TLimit
///         are considered to be valid.
/// @headerfile embxx/comms/field/BasicEnumValue.h
template <typename TEnum,
          typename TTraits,
          std::size_t TLen = sizeof(std::uint8_t),
          TEnum TLimit = static_cast<TEnum>(std::numeric_limits<typename embxx::util::SizeToType<TLen>::Type>::max())>
class BasicEnumValue
{
    static_assert(std::is_enum<TEnum>::value, "TEnum must be enum type");

public:
    /// @brief Definition of underlying BasicIntValue field type
    typedef
        BasicIntValue<
            typename util::SizeToType<TLen>::Type,
            TTraits,
            TLen
        > IntValueField;

    /// @brief Type of the stored value
    typedef TEnum ValueType;

    /// @brief Serialised Type
    typedef typename IntValueField::SerialisedType SerialisedType;

    /// @brief Field traits
    typedef TTraits Traits;

    /// @brief Data endianness
    typedef typename Traits::Endianness Endianness;

    /// @brief Length of serialised data
    static const std::size_t SerialisedLen = TLen;

    /// @brief Limit of valid values
    static const ValueType LimitValue = TLimit;

    /// @brief Default constructor.
    /// @brief Initial value is equal to LimitValue
    BasicEnumValue();

    /// @brief Constructor
    explicit BasicEnumValue(ValueType value);

    /// @brief Copy constructor is default
    BasicEnumValue(const BasicEnumValue&) = default;

    /// @brief Destructor is default
    ~BasicEnumValue() = default;

    /// @brief Copy assignment is default
    BasicEnumValue& operator=(const BasicEnumValue&) = default;

    /// @brief Retrieve underlying BasicIntValue field.
    const IntValueField asIntValueField() const;

    /// @copydoc BasicIntValue::getValue()
    const ValueType getValue() const;

    /// @copydoc BasicIntValue::setValue()
    void setValue(ValueType value);

    /// @copydoc BasicIntValue::getSerialisedValue()
    const SerialisedType getSerialisedValue() const;

    /// @copydoc BasicIntValue::setSerialisedValue()
    void setSerialisedValue(SerialisedType value);

    /// @copydoc BasicIntValue::toSerialised()
    static constexpr const SerialisedType toSerialised(ValueType value);

    /// @copydoc BasicIntValue::fromSerialised()
    static constexpr const ValueType fromSerialised(SerialisedType value);

    /// @copydoc BasicIntValue::length()
    static constexpr std::size_t length();

    /// @copydoc BasicIntValue::read()
    template <typename TIter>
    ErrorStatus read(TIter& iter, std::size_t size);

    /// @copydoc BasicIntValue::write()
    template <typename TIter>
    ErrorStatus write(TIter& iter, std::size_t size) const;

    /// @brief Check whether value is in range [0, ValueLimit).
    bool isValid() const;

private:
    IntValueField intValue_;
};

// Implementation

/// @brief Equality comparison operator.
/// @related BasicEnumValue
template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
bool operator==(
    const BasicEnumValue<TEnum, TTraits, TLen, TLimit>& field1,
    const BasicEnumValue<TEnum, TTraits, TLen, TLimit>& field2)
{
    return field1.asIntValueField() == field2.asIntValueField();
}

/// @brief Non-equality comparison operator.
/// @related BasicEnumValue
template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
bool operator!=(
    const BasicEnumValue<TEnum, TTraits, TLen, TLimit>& field1,
    const BasicEnumValue<TEnum, TTraits, TLen, TLimit>& field2)
{
    return field1.asIntValueField() != field2.asIntValueField();
}

/// @brief Equivalence comparison operator.
/// @related BasicEnumValue
template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
bool operator<(
    const BasicEnumValue<TEnum, TTraits, TLen, TLimit>& field1,
    const BasicEnumValue<TEnum, TTraits, TLen, TLimit>& field2)
{
    return field1.asIntValueField() < field2.asIntValueField();
}

/// @}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
BasicEnumValue<TEnum, TTraits, TLen, TLimit>::BasicEnumValue()
    : intValue_(static_cast<typename IntValueField::ValueType>(TLimit))
{
}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
BasicEnumValue<TEnum, TTraits, TLen, TLimit>::BasicEnumValue(ValueType value)
    : intValue_(static_cast<typename IntValueField::ValueType>(value))
{
}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
const typename BasicEnumValue<TEnum, TTraits, TLen, TLimit>::IntValueField
BasicEnumValue<TEnum, TTraits, TLen, TLimit>::asIntValueField() const
{
    return intValue_;
}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
const typename BasicEnumValue<TEnum, TTraits, TLen, TLimit>::ValueType
BasicEnumValue<TEnum, TTraits, TLen, TLimit>::getValue() const
{
    return static_cast<ValueType>(intValue_.getValue());
}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
void BasicEnumValue<TEnum, TTraits, TLen, TLimit>::setValue(ValueType value)
{
    intValue_.setValue(static_cast<typename IntValueField::ValueType>(value));
}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
const typename BasicEnumValue<TEnum, TTraits, TLen, TLimit>::SerialisedType
BasicEnumValue<TEnum, TTraits, TLen, TLimit>::getSerialisedValue() const
{
    return intValue_.getSerialisedValue();
}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
void BasicEnumValue<TEnum, TTraits, TLen, TLimit>::setSerialisedValue(
    SerialisedType value)
{
    intValue_.setSerialisedValue(value);
}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
constexpr
const typename BasicEnumValue<TEnum, TTraits, TLen, TLimit>::SerialisedType
BasicEnumValue<TEnum, TTraits, TLen, TLimit>::toSerialised(ValueType value)
{
    return IntValueField::toSerialised(
        static_cast<typename IntValueField::ValueType>(value));
}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
constexpr
const typename BasicEnumValue<TEnum, TTraits, TLen, TLimit>::ValueType
BasicEnumValue<TEnum, TTraits, TLen, TLimit>::fromSerialised(SerialisedType value)
{
    return static_cast<ValueType>(IntValueField::fromSerialised(value));
}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
constexpr std::size_t BasicEnumValue<TEnum, TTraits, TLen, TLimit>::length()
{
    return IntValueField::length();
}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
template <typename TIter>
ErrorStatus BasicEnumValue<TEnum, TTraits, TLen, TLimit>::read(
    TIter& iter,
    std::size_t size)
{
    return intValue_.read(iter, size);
}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
template <typename TIter>
ErrorStatus BasicEnumValue<TEnum, TTraits, TLen, TLimit>::write(
    TIter& iter,
    std::size_t size) const
{
    return intValue_.write(iter, size);
}

template <typename TEnum,
          typename TTraits,
          std::size_t TLen,
          TEnum TLimit>
bool BasicEnumValue<TEnum, TTraits, TLen, TLimit>::isValid() const
{
    return (getValue() < LimitValue);
}

}  // namespace field

}  // namespace comms

}  // namespace embxx





