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
#include "embxx/io/iosimple.h"
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
/// @headerfile embxx/comms/field.h
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
    static constexpr std::size_t getLength();

    /// @brief Read the serialised field value from the stream buffer.
    /// @param[in, out] buf Input stream buffer.
    /// @param[in] size Size of the data in the stream buffer.
    /// @return Status of the read operation.
    /// @pre Value of provided "size" must be less than or equal to
    ///      available data in the buffer
    /// @pre @code size <= buf.in_avail() @endcode
    /// @post The internal (std::ios_base::in) pointer of the stream buffer
    ///       will be advanced by the number of bytes was actually read.
    ///       In case of an error, it will provide an information to the caller
    ///       about the place the error was recognised.
    ErrorStatus read(std::streambuf& buf, std::size_t size);

    /// @brief Write the serialised field value to the stream buffer.
    /// @param[in, out] buf Output stream buffer.
    /// @param[in] size Size of the buffer, message data must fit it.
    /// @return Status of the write operation.
    /// @pre Value of provided "size" must be less than or equal to
    ///      available space in the buffer.
    /// @pre Available space in the buffer is greater of equal to the
    ///      serialisation length of the field.
    /// @post The internal (std::ios_base::out) pointer of the stream buffer
    ///       will be advanced by the number of bytes was actually written.
    ///       In case of an error, it will provide an information to the caller
    ///       about the place the error was recognised.
    ErrorStatus write(std::streambuf& buf, std::size_t size) const;

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
constexpr std::size_t BasicIntValue<T, TTraits, TLen, TOff>::getLength()
{
    return SerialisedLen;
}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
ErrorStatus BasicIntValue<T, TTraits, TLen, TOff>::read(
    std::streambuf& buf,
    std::size_t size)
{
    GASSERT(size <= buf.in_avail());
    if (size < SerialisedLen) {
        return ErrorStatus::NotEnoughData;
    }

    auto serialisedValue =
        io::getData<SerialisedType, SerialisedLen>(
            buf,
            Endianness());
    setSerialisedValue(serialisedValue);
    return ErrorStatus::Success;
}

template <typename T,
          typename TTraits,
          std::size_t TLen,
          typename util::IntegralPromotion<T>::Type TOff>
ErrorStatus BasicIntValue<T, TTraits, TLen, TOff>::write(
    std::streambuf& buf,
    std::size_t size) const
{

#ifndef NDEBUG
    auto firstPos = buf.pubseekoff(0, std::ios_base::cur, std::ios_base::out);
    auto lastPos = buf.pubseekoff(0, std::ios_base::end, std::ios_base::out);
    buf.pubseekpos(firstPos, std::ios_base::out);
    auto diff = static_cast<decltype(size)>(lastPos - firstPos);
    GASSERT(size <= diff);
#endif // #ifndef NDEBUG

    GASSERT(getLength() <= size);
    static_cast<void>(size);

    io::putData<SerialisedLen>(getSerialisedValue(), buf, Endianness());
    return ErrorStatus::Success;
}


}  // namespace field

}  // namespace comms

}  // namespace embxx
