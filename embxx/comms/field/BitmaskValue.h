//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file BitmaskValue.h
/// This file contains definition of bitmask value field that
/// can be used in message definition.

#pragma once

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

/// @brief Defines "Bitmask Value Field".
/// @details The class uses BasicIntValue as its underlying type while providing
///          additional API functions to access/set/clear bits in the bitmask.
/// @tparam TLen Length of serialised data in bytes.
/// @tparam TTraits Various behavioural traits relevant for the field.
///         Currently the only trait that is required for this class is
///         Endianness. The traits class/struct must typedef either
///         embxx::comms::traits::endian::Big or
///         embxx::comms::traits::endian::Little to Endianness.
/// @headerfile embxx/comms/field/BasicIntValue.h
/// @headerfile embxx/comms/field.h
template <std::size_t TLen,
          typename TTraits>
class BitmaskValue
{
public:
    /// @brief Definition of underlying BasicIntValue field type
    typedef
        BasicIntValue<
            typename util::SizeToType<TLen>::Type,
            TTraits,
            TLen
        > IntValueField;

    /// @brief Type of the stored value
    typedef typename IntValueField::ValueType ValueType;

    /// @brief Serialised Type
    typedef typename IntValueField::SerialisedType SerialisedType;

    /// @brief Field traits
    typedef TTraits Traits;

    /// @brief Data endianness
    typedef typename Traits::Endianness Endianness;

    /// @brief Length of serialised data
    static const std::size_t SerialisedLen = TLen;

    /// @brief Default constructor.
    /// @brief Initial bitmask has all bits cleared (equals 0)
    BitmaskValue();

    /// @brief Constructor
    explicit BitmaskValue(ValueType value);

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

    /// @copydoc BasicIntValue::getLength()
    static constexpr std::size_t getLength();

    /// @copydoc BasicIntValue::read()
    ErrorStatus read(std::streambuf& buf, std::size_t size);

    /// @copydoc BasicIntValue::write()
    ErrorStatus write(std::streambuf& buf, std::size_t size) const;

    /// @brief Check whether all bits from provided mask are set.
    /// @param[in] mask Mask to check against
    /// @return true in case all the bits are set, false otherwise
    bool hasAllBitsSet(ValueType mask) const;

    /// @brief Check whether any bits from provided mask are set.
    /// @param[in] mask Mask to check against
    /// @return true in case at least one of the bits is set, false otherwise.
    bool hasAnyBitsSet(ValueType mask) const;

    /// @brief Set all the provided bits.
    /// @details Equivalent to @code setValue(getValue() | mask); @endcode
    /// @param[in] mask Mask of bits to set.
    void setBits(ValueType mask);

    /// @brief Set all the provided bits.
    /// @details Equivalent to @code setValue(getValue() & (~mask)); @endcode
    /// @param[in] mask Mask of bits to clear.
    void clearBits(ValueType mask);

private:
    IntValueField intValue_;
};

// Implementation

/// @brief Equality comparison operator.
/// @related BitmaskValue
template <std::size_t TLen,
          typename TTraits>
bool operator==(
    const BitmaskValue<TLen, TTraits>& field1,
    const BitmaskValue<TLen, TTraits>& field2)
{
    return field1.asIntValueField() == field2.asIntValueField();
}

/// @brief Equivalence comparison operator.
/// @related BitmaskValue
template <std::size_t TLen,
          typename TTraits>
bool operator<(
    const BitmaskValue<TLen, TTraits>& field1,
    const BitmaskValue<TLen, TTraits>& field2)
{
    return field1.asIntValueField() < field2.asIntValueField();
}

/// @}

template <std::size_t TLen,
          typename TTraits>
BitmaskValue<TLen, TTraits>::BitmaskValue()
{
}

template <std::size_t TLen,
          typename TTraits>
BitmaskValue<TLen, TTraits>::BitmaskValue(ValueType value)
    : intValue_(value)
{
}

template <std::size_t TLen,
          typename TTraits>
const typename BitmaskValue<TLen, TTraits>::IntValueField
BitmaskValue<TLen, TTraits>::asIntValueField() const
{
    return intValue_;
}

template <std::size_t TLen,
          typename TTraits>
const typename BitmaskValue<TLen, TTraits>::ValueType
BitmaskValue<TLen, TTraits>::getValue() const
{
    return intValue_.getValue();
}

template <std::size_t TLen,
          typename TTraits>
void BitmaskValue<TLen, TTraits>::setValue(ValueType value)
{
    intValue_.setValue(value);
}

template <std::size_t TLen,
          typename TTraits>
const typename BitmaskValue<TLen, TTraits>::SerialisedType
BitmaskValue<TLen, TTraits>::getSerialisedValue() const
{
    return intValue_.getSerialisedValue();
}

template <std::size_t TLen,
          typename TTraits>
void BitmaskValue<TLen, TTraits>::setSerialisedValue(
    SerialisedType value)
{
    intValue_.setSerialisedValue(value);
}

template <std::size_t TLen,
          typename TTraits>
constexpr
const typename BitmaskValue<TLen, TTraits>::SerialisedType
BitmaskValue<TLen, TTraits>::toSerialised(ValueType value)
{
    return IntValueField::toSerialised(value);
}

template <std::size_t TLen,
          typename TTraits>
constexpr
const typename BitmaskValue<TLen, TTraits>::ValueType
BitmaskValue<TLen, TTraits>::fromSerialised(SerialisedType value)
{
    return IntValueField::fromSerialised(value);
}

template <std::size_t TLen,
          typename TTraits>
constexpr std::size_t BitmaskValue<TLen, TTraits>::getLength()
{
    return IntValueField::getLength();
}

template <std::size_t TLen,
          typename TTraits>
ErrorStatus BitmaskValue<TLen, TTraits>::read(
    std::streambuf& buf,
    std::size_t size)
{
    return intValue_.read(buf, size);
}

template <std::size_t TLen,
          typename TTraits>
ErrorStatus BitmaskValue<TLen, TTraits>::write(
    std::streambuf& buf,
    std::size_t size) const
{
    return intValue_.write(buf, size);
}

template <std::size_t TLen,
          typename TTraits>
bool BitmaskValue<TLen, TTraits>::hasAllBitsSet(ValueType mask) const
{
    return (getValue() & mask) == mask;
}

template <std::size_t TLen,
          typename TTraits>
bool BitmaskValue<TLen, TTraits>::hasAnyBitsSet(ValueType mask) const
{
    return (getValue() & mask) != 0;
}


template <std::size_t TLen,
          typename TTraits>
void BitmaskValue<TLen, TTraits>::setBits(ValueType mask)
{
    setValue(getValue() | mask);
}

template <std::size_t TLen,
          typename TTraits>
void BitmaskValue<TLen, TTraits>::clearBits(ValueType mask)
{
    setValue(getValue() & (~mask));
}

}  // namespace field

}  // namespace comms

}  // namespace embxx

