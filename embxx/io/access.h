//
// Copyright 2012 (C). Alex Robenko. All rights reserved.
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

/// @file embxx/io/access.h
/// Simple data access module.
/// Provides an ability to read/write serialised integral types using iterators.

#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "traits.h"

namespace embxx
{

namespace io
{
/// @addtogroup io
/// @{

/// @brief Write (serialise) integral value with MSB first (Big endian).
/// @tparam T Type of the value. Must be integral type.
/// @tparam TIter Type of the output iterator.
/// @param[in] value Value to write.
/// @param[in, out] iter Output iterator of character stream.
/// @pre The iterator must be valid.
/// @pre The iterator must provide pre-increment operator;
/// @post The iterator is advanced.
/// @note Exception guarantee: Basic
template <typename T, typename TIter>
void writeBig(T value, TIter& iter);

/// @brief Write (serialise) part of the integral value with MSB first (Big endian).
/// @tparam TSize Size (in number of bytes) to write.
/// @tparam T Type of the value. Must be integral type.
/// @tparam TIter Type of the output iterator.
/// @param[in] value Value to write.
/// @param[in, out] iter Output iterator of character stream.
/// @pre The iterator must be valid.
/// @pre The iterator must provide pre-increment operator;
/// @pre (TSize <= sizeof(T))
/// @post The iterator is advanced.
/// @note Exception guarantee: Basic
template <std::size_t TSize, typename T, typename TIter>
void writeBig(T value, TIter& iter);

/// @brief Read (de-serialise) integral value with MSB first (Big endian).
/// @tparam T Type of the value. Must be integral type.
/// @tparam TIter Type of the input iterator.
/// @param[in, out] iter Input iterator of character stream.
/// @return Value read from character stream.
/// @pre The iterator must be valid.
/// @pre The iterator must provide pre-increment operator;
/// @post The iterator is advanced.
/// @note Exception guarantee: Basic
template <typename T, typename TIter>
T readBig(TIter& iter);

/// @brief Read (de-serialise) integral value with MSB first (Big endian).
/// @tparam T Type of the value. Must be integral type.
/// @tparam TSize Size (in number of bytes) to read.
/// @tparam TIter Type of the input iterator.
/// @param[in, out] iter Input iterator of character stream.
/// @return Value read from character stream.
/// @pre The iterator must be valid.
/// @pre The iterator must provide pre-increment operator;
/// @pre (TSize <= sizeof(T))
/// @post The iterator is advanced.
/// @note Exception guarantee: Basic
template <typename T, std::size_t TSize, typename TIter>
T readBig(TIter& iter);

/// @brief Write (serialise) integral value with LSB first (Little endian).
/// @tparam T Type of the value. Must be integral type.
/// @tparam TIter Type of the output iterator.
/// @param[in] value Value to write.
/// @param[in, out] iter Output iterator of character stream.
/// @pre The iterator must be valid.
/// @pre The iterator must provide pre-increment operator;
/// @post The iterator is advanced.
/// @note Exception guarantee: Basic
template <typename T, typename TIter>
void writeLittle(T value, TIter& iter);

/// @brief Write (serialise) part of the integral value with LSB first (Little endian).
/// @tparam TSize Size (in number of bytes) to write.
/// @tparam T Type of the value. Must be integral type.
/// @tparam TIter Type of the output iterator.
/// @param[in] value Value to write.
/// @param[in, out] iter Output iterator of character stream.
/// @pre The iterator must be valid.
/// @pre The iterator must provide pre-increment operator;
/// @pre (TSize <= sizeof(T))
/// @post The iterator is advanced.
/// @note Exception guarantee: Basic
template <std::size_t TSize, typename T, typename TIter>
void writeLittle(T value, TIter& iter);

/// @brief Read (de-serialise) integral value with LSB first (Little endian).
/// @tparam T Type of the value. Must be integral type.
/// @tparam TIter Type of the input iterator.
/// @param[in, out] iter Input iterator of character stream.
/// @return Value read from character stream.
/// @pre The iterator must be valid.
/// @pre The iterator must provide pre-increment operator;
/// @post The iterator is advanced.
/// @note Exception guarantee: Basic
template <typename T, typename TIter>
T readLittle(TIter& iter);

/// @brief Read (de-serialise) integral value with LSB first (Little endian).
/// @tparam T Type of the value. Must be integral type.
/// @tparam TSize Size (in number of bytes) to read.
/// @tparam TIter Type of the input iterator.
/// @param[in, out] iter Input iterator of character stream.
/// @return Value read from character stream.
/// @pre The iterator must be valid.
/// @pre The iterator must provide pre-increment operator;
/// @pre (TSize <= sizeof(T))
/// @post The iterator is advanced.
/// @note Exception guarantee: Basic
template <typename T, std::size_t TSize, typename TIter>
T readLittle(TIter& iter);

/// Same as writeBig<T, TIter>(value, iter);
template <typename T, typename TIter>
void writeData(
    T value,
    TIter& iter,
    const traits::endian::Big& endian);

/// Same as writeBig<TSize, T, TIter>(value, iter)
template <std::size_t TSize, typename T, typename TIter>
void writeData(
    T value,
    TIter& iter,
    const traits::endian::Big& endian);

/// Same as writeLittle<T, TIter>(value, iter)
template <typename T, typename TIter>
void writeData(
    T value,
    TIter& iter,
    const traits::endian::Little& endian);

/// Same as writeLittle<TSize, T, TIter>(value, iter)
template <std::size_t TSize, typename T, typename TIter>
void writeData(
    T value,
    TIter& iter,
    const traits::endian::Little& endian);

/// Same as readBig<T, TIter>(iter)
template <typename T, typename TIter>
T readData(TIter& iter, const traits::endian::Big& endian);

/// Same as readBig<TSize, T, TIter>(iter)
template <typename T, std::size_t TSize, typename TIter>
T readData(TIter& iter, const traits::endian::Big& endian);

/// Same as readLittle<T, TIter>(iter)
template <typename T, typename TIter>
T readData(TIter& iter, const traits::endian::Little& endian);

/// Same as readLittle<TSize, T>(iter)
template <typename T, std::size_t TSize, typename TIter>
T readData(TIter& iter, const traits::endian::Little& endian);

/// @}

// Implementation part

namespace details
{

template <typename T, std::size_t TSize, typename TByteType>
class SignExt
{
public:

    static T value(T value)
    {
        typedef typename std::decay<T>::type ValueType;
        ValueType mask =
            (static_cast<ValueType>(1) << ((TSize * std::numeric_limits<TByteType>::digits) - 1));
        if (value & mask) {
            return value | (~((mask << 1) - 1));
        }
        return value;
    }
};

template <typename TByteType>
class SignExt<std::int8_t, sizeof(std::int8_t), TByteType>
{
public:
    static std::int8_t value(std::int8_t value)
    {
        return value;
    }
};

template <typename TByteType>
class SignExt<std::int16_t, sizeof(std::int16_t), TByteType>
{
public:
    static std::int16_t value(std::int16_t value)
    {
        return value;
    }
};

template <typename TByteType>
class SignExt<std::int32_t, sizeof(std::int32_t), TByteType>
{
public:
    static std::int32_t value(std::int32_t value)
    {
        return value;
    }
};

template <typename TByteType>
class SignExt<std::int64_t, sizeof(std::int64_t), TByteType>
{
public:
    static std::int64_t value(std::int64_t value)
    {
        return value;
    }
};

template <typename TIter, bool TIsPointer>
struct ByteTypeRetriever;

template <typename TIter>
struct ByteTypeRetriever<TIter, true>
{
    typedef typename std::decay<decltype(*(TIter()))>::type Type;
};

template <typename TIter>
struct ByteTypeRetriever<TIter, false>
{
    typedef typename std::decay<TIter>::type DecayedIter;
    typedef typename DecayedIter::value_type Type;
};

template <typename TContainer>
struct ByteTypeRetriever<std::back_insert_iterator<TContainer>, false>
{
    typedef typename std::decay<TContainer>::type DecayedContainer;
    typedef typename DecayedContainer::value_type Type;
};

template <typename TContainer>
struct ByteTypeRetriever<std::front_insert_iterator<TContainer>, false>
{
    typedef typename std::decay<TContainer>::type DecayedContainer;
    typedef typename DecayedContainer::value_type Type;
};

template <typename TIter>
using ByteType = typename ByteTypeRetriever<TIter, std::is_pointer<TIter>::value>::Type;


template <typename T, typename TIter>
void writeBig(T value, std::size_t size, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    typedef typename std::make_unsigned<ValueType>::type UnsignedType;
    typedef ByteType<TIter> ByteType;

    UnsignedType unsignedValue = static_cast<UnsignedType>(value);
    std::size_t remainingSize = size;
    while (remainingSize > 0) {
        std::size_t remaingShift =
            ((remainingSize - 1) * std::numeric_limits<ByteType>::digits);
        auto byte = static_cast<ByteType>(unsignedValue >> remaingShift);
        *iter = byte;
        ++iter;
        --remainingSize;
    }
}

template <typename T, typename TIter>
T readBig(std::size_t size, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    typedef typename std::make_unsigned<ValueType>::type UnsignedType;
    typedef ByteType<TIter> ByteType;

    UnsignedType value = 0;
    std::size_t remainingSize = size;
    while (remainingSize > 0) {
        auto byte = *iter;
        value <<= std::numeric_limits<ByteType>::digits;
        value |= byte;
        ++iter;
        --remainingSize;
    }

    return static_cast<T>(static_cast<ValueType>(value));
}

template <typename T, typename TIter>
void writeLittle(T value, std::size_t size, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    typedef typename std::make_unsigned<ValueType>::type UnsignedType;
    typedef ByteType<TIter> ByteType;

    std::size_t remainingSize = size;
    UnsignedType unsignedValue = static_cast<UnsignedType>(value);
    while (remainingSize > 0) {
        std::size_t remaingShift =
            ((size - remainingSize) * std::numeric_limits<ByteType>::digits);

        auto byte = static_cast<ByteType>(unsignedValue >> remaingShift);
        *iter = byte;
        ++iter;
        --remainingSize;
    }
}

template <typename T, typename TIter>
T readLittle(std::size_t size, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    typedef typename std::make_unsigned<ValueType>::type UnsignedType;
    typedef ByteType<TIter> ByteType;

    UnsignedType value = 0;
    std::size_t remainingSize = size;
    while (remainingSize > 0) {
        auto byte = *iter;
        value |= static_cast<UnsignedType>(byte) <<
            ((size - remainingSize) * std::numeric_limits<ByteType>::digits);
        ++iter;
        --remainingSize;
    }

    return static_cast<T>(static_cast<ValueType>(value));
}

}  // namespace details

template <typename T, typename TIter>
void writeBig(T value, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    writeBig<sizeof(ValueType)>(static_cast<ValueType>(value), iter);
}

template <std::size_t TSize, typename T, typename TIter>
void writeBig(T value, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;

    static_assert(TSize <= sizeof(ValueType), "Precondition failure");
    details::writeBig(static_cast<ValueType>(value), TSize, iter);
}

template <typename T, typename TIter>
T readBig(TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    return static_cast<T>(readBig<ValueType, sizeof(ValueType)>(iter));
}

template <typename T, std::size_t TSize, typename TIter>
T readBig(TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    typedef details::ByteType<TIter> ByteType;

    static_assert(TSize <= sizeof(ValueType), "Precondition failure");

    auto retval = details::readBig<ValueType>(TSize, iter);
    if (std::is_signed<ValueType>::value) {
        retval = details::SignExt<decltype(retval), TSize, ByteType>::value(retval);
    }
    return static_cast<T>(retval);
}

template <typename T, typename TIter>
void writeLittle(T value, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    writeLittle<sizeof(ValueType)>(static_cast<ValueType>(value), iter);
}

template <std::size_t TSize, typename T, typename TIter>
void writeLittle(T value, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    static_assert(TSize <= sizeof(ValueType), "Precondition failure");
    details::writeLittle(static_cast<ValueType>(value), TSize, iter);
}

template <typename T, typename TIter>
T readLittle(TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    return static_cast<T>(readLittle<ValueType, sizeof(ValueType)>(iter));
}

template <typename T, std::size_t TSize, typename TIter>
T readLittle(TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    typedef details::ByteType<TIter> ByteType;
    static_assert(TSize <= sizeof(ValueType), "Precondition failure");
    auto retval = details::readLittle<ValueType>(TSize, iter);
    if (std::is_signed<ValueType>::value) {
        retval = details::SignExt<T, TSize, ByteType>::value(retval);
    }
    return retval;
}

template <typename T, typename TIter>
void writeData(
    T value,
    TIter& iter,
    const traits::endian::Big& endian)
{
    static_cast<void>(endian);
    writeBig(value, iter);
}

template <std::size_t TSize, typename T, typename TIter>
void writeData(
    T value,
    TIter& iter,
    const traits::endian::Big& endian)
{
    static_cast<void>(endian);
    writeBig<TSize>(value, iter);
}

template <typename T, typename TIter>
void writeData(
    T value,
    TIter& iter,
    const traits::endian::Little& endian)
{
    static_cast<void>(endian);
    writeLittle(value, iter);
}

template <std::size_t TSize, typename T, typename TIter>
void writeData(
    T value,
    TIter& iter,
    const traits::endian::Little& endian)
{
    static_cast<void>(endian);
    return writeLittle<TSize>(value, iter);
}

template <typename T, typename TIter>
T readData(TIter& iter, const traits::endian::Big& endian)
{
    static_cast<void>(endian);
    return readBig<T>(iter);
}

template <typename T, std::size_t TSize, typename TIter>
T readData(TIter& iter, const traits::endian::Big& endian)
{
    static_cast<void>(endian);
    return readBig<T, TSize>(iter);
}

template <typename T, typename TIter>
T readData(TIter& iter, const traits::endian::Little& endian)
{
    static_cast<void>(endian);
    return readLittle<T>(iter);
}

template <typename T, std::size_t TSize, typename TIter>
T readData(TIter& iter, const traits::endian::Little& endian)
{
    static_cast<void>(endian);
    return readLittle<T, TSize>(iter);
}


}  // namespace io

}  // namespace embxx
