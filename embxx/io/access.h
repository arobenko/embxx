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
/// @pre value is of integral type.
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
/// @pre value is of integral type.
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
/// @pre value is of integral type.
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
/// @pre value is of integral type.
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
/// @pre value is of integral type.
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
/// @pre value is of integral type.
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
/// @pre value is of integral type.
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
/// @pre value is of integral type.
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

/// Same as readLittle<TSize, T, TIter>(iter)
template <typename T, std::size_t TSize, typename TIter>
T readData(TIter& iter, const traits::endian::Little& endian);

/// @}

// Implementation part

namespace details
{

template <typename T, bool TMakeIntSize>
struct TypeOptimiser;

template <typename T>
struct TypeOptimiser<T, false>
{
    typedef typename std::decay<T>::type Type;
};

template <typename T>
struct TypeOptimiser<T, true>
{
    typedef typename std::conditional<std::is_signed<T>::value, int, unsigned>::type Type;
};

template <typename T>
using OptimisedValueType = typename TypeOptimiser<T, sizeof(T) < sizeof(int)>::Type;

template <typename TUnsignedByteType, typename T>
typename std::decay<T>::type signExtCommon(T value, std::size_t size)
{
    typedef typename std::decay<T>::type ValueType;
    static_assert(std::is_unsigned<ValueType>::value, "Type T must be unsigned");
    static_assert(std::is_unsigned<TUnsignedByteType>::value,
                                "Type TUnsignedByteType must be unsigned");

    static const std::size_t BinDigits =
        std::numeric_limits<TUnsignedByteType>::digits;
    static_assert(BinDigits % 8 == 0, "Byte size assumption is not valid");

    ValueType mask =
        (static_cast<ValueType>(1) << ((size * BinDigits) - 1));
    if (value & mask) {
        return value | (~((mask << 1) - 1));
    }
    return value;
}

template <typename T, std::size_t TSize, typename TByteType>
class SignExt
{
public:

    static typename std::decay<T>::type value(T value)
    {
        typedef typename std::decay<T>::type ValueType;
        typedef typename std::make_unsigned<ValueType>::type UnsignedValueType;
        static_assert(std::is_integral<T>::value, "T must be integer type");
        typedef typename std::make_unsigned<TByteType>::type UnsignedByteType;

        auto castedValue = static_cast<UnsignedValueType>(value);
        return static_cast<ValueType>(
            signExtCommon<UnsignedByteType>(castedValue, TSize));
    }
};

template <typename T, typename TByteType>
class SignExt<T, sizeof(typename std::decay<T>::type), TByteType>
{
public:
    static typename std::decay<T>::type value(T value)
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
void writeBigUnsigned(T value, std::size_t size, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    static_assert(std::is_unsigned<ValueType>::value, "T type must be unsigned");
    static_assert(std::is_integral<ValueType>::value, "T must be integral type");

    typedef ByteType<TIter> ByteType;
    typedef typename std::make_unsigned<ByteType>::type UnsignedByteType;
    static const std::size_t BinDigits =
        std::numeric_limits<UnsignedByteType>::digits;
    static_assert(BinDigits % 8 == 0, "Byte size assumption is not valid");

    std::size_t remainingSize = size;
    while (remainingSize > 0) {
        std::size_t remaingShift = ((remainingSize - 1) * BinDigits);
        auto byte = static_cast<ByteType>(value >> remaingShift);
        *iter = byte;
        ++iter;
        --remainingSize;
    }
}

template <typename T, typename TIter>
void writeLittleUnsigned(T value, std::size_t size, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    static_assert(std::is_integral<ValueType>::value, "T must be integral type");
    static_assert(std::is_unsigned<ValueType>::value, "T type must be unsigned");

    typedef ByteType<TIter> ByteType;
    typedef typename std::make_unsigned<ByteType>::type UnsignedByteType;
    static const std::size_t BinDigits =
        std::numeric_limits<UnsignedByteType>::digits;
    static_assert(BinDigits % 8 == 0, "Byte size assumption is not valid");

    std::size_t remainingSize = size;
    while (remainingSize > 0) {
        std::size_t remaingShift = ((size - remainingSize) * BinDigits);

        auto byte = static_cast<ByteType>(value >> remaingShift);
        *iter = byte;
        ++iter;
        --remainingSize;
    }
}

template <typename TEndian>
struct WriteUnsignedFuncWrapper;

template <>
struct WriteUnsignedFuncWrapper<traits::endian::Big>
{
    template <typename T, typename TIter>
    static void write(T value, std::size_t size, TIter& iter)
    {
        writeBigUnsigned(value, size, iter);
    }
};

template <>
struct WriteUnsignedFuncWrapper<traits::endian::Little>
{
    template <typename T, typename TIter>
    static void write(T value, std::size_t size, TIter& iter)
    {
        writeLittleUnsigned(value, size, iter);
    }
};

template <typename TEndian, typename T, typename TIter>
void write(T value, std::size_t size, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    static_assert(std::is_integral<ValueType>::value, "T must be integral type");
    typedef typename std::make_unsigned<ValueType>::type UnsignedType;
    UnsignedType unsignedValue = static_cast<UnsignedType>(value);
    WriteUnsignedFuncWrapper<TEndian>::write(unsignedValue, size, iter);
}

template <typename TEndian, typename T, typename TIter>
void writeRandomAccess(T value, std::size_t size, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;

    static_assert(std::is_integral<ValueType>::value, "T must be integral type");
    static_assert(
        std::is_same<
            typename std::iterator_traits<TIter>::iterator_category,
            std::random_access_iterator_tag
        >::value,
        "TIter must be random access iterator");

    typedef ByteType<TIter> ByteType;
    typedef typename std::make_unsigned<ByteType>::type UnsignedByteType;
    static_assert(!std::is_const<UnsignedByteType>::value, "Value must be updatable");

    auto startPtr = reinterpret_cast<UnsignedByteType*>(&(*iter));
    auto endPtr = startPtr;
    write<TEndian>(value, size, endPtr);
    iter += (endPtr - startPtr);
}

template <typename TEndian, bool TIsPointerToUnsigned>
struct WriteRandomAccessHelper;

template <typename TEndian>
struct WriteRandomAccessHelper<TEndian, false>
{
    template <typename T, typename TIter>
    static void write(T value, std::size_t size, TIter& iter)
    {
        writeRandomAccess<TEndian>(value, size, iter);
    }
};

template <typename TEndian>
struct WriteRandomAccessHelper<TEndian, true>
{
    template <typename T, typename TIter>
    static void write(T value, std::size_t size, TIter& iter)
    {
        details::write<TEndian>(value, size, iter);
    }
};

template <typename TEndian, bool TIsRandomAccess>
struct WriteHelper;

template <typename TEndian>
struct WriteHelper<TEndian, false>
{
    template <typename T, typename TIter>
    static void write(T value, std::size_t size, TIter& iter)
    {
        details::write<TEndian>(value, size, iter);
    }
};

template <typename TEndian>
struct WriteHelper<TEndian, true>
{
    template <typename T, typename TIter>
    static void write(T value, std::size_t size, TIter& iter)
    {
        typedef ByteType<TIter> ByteType;
        static const bool IsPointerToUnsigned =
            std::is_pointer<TIter>::value &&
            std::is_unsigned<ByteType>::value;
        return WriteRandomAccessHelper<TEndian, IsPointerToUnsigned>::write(value, size, iter);
    }
};

template <typename T, typename TIter>
T readBigUnsigned(std::size_t size, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    static_assert(std::is_integral<ValueType>::value, "T must be integral type");
    static_assert(std::is_unsigned<ValueType>::value, "T type must be unsigned");

    typedef ByteType<TIter> ByteType;
    typedef typename std::make_unsigned<ByteType>::type UnsignedByteType;
    static const std::size_t BinDigits =
        std::numeric_limits<UnsignedByteType>::digits;
    static_assert(BinDigits % 8 == 0, "Byte size assumption is not valid");

    ValueType value = 0;
    std::size_t remainingSize = size;
    while (remainingSize > 0) {
        auto byte = *iter;
        value <<= BinDigits;
        value |= static_cast<decltype(value)>(static_cast<UnsignedByteType>(byte));
        ++iter;
        --remainingSize;
    }
    return value;
}

template <typename T, typename TIter>
T readLittleUnsigned(std::size_t size, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;
    static_assert(std::is_integral<ValueType>::value, "T must be integral type");
    static_assert(std::is_unsigned<ValueType>::value, "T type must be unsigned");

    typedef ByteType<TIter> ByteType;
    typedef typename std::make_unsigned<ByteType>::type UnsignedByteType;
    static const std::size_t BinDigits =
        std::numeric_limits<UnsignedByteType>::digits;
    static_assert(BinDigits % 8 == 0, "Byte size assumption is not valid");

    ValueType value = 0;
    std::size_t remainingSize = size;
    while (remainingSize > 0) {
        auto byte = *iter;
        value |= static_cast<ValueType>(static_cast<UnsignedByteType>(byte)) <<
            ((size - remainingSize) * BinDigits);
        ++iter;
        --remainingSize;
    }

    return static_cast<T>(value);
}

template <typename TEndian>
struct ReadUnsignedFuncWrapper;

template <>
struct ReadUnsignedFuncWrapper<traits::endian::Big>
{
    template <typename T, typename TIter>
    static T read(std::size_t size, TIter& iter)
    {
        return readBigUnsigned<T>(size, iter);
    }
};

template <>
struct ReadUnsignedFuncWrapper<traits::endian::Little>
{
    template <typename T, typename TIter>
    static T read(std::size_t size, TIter& iter)
    {
        return readLittleUnsigned<T>(size, iter);
    }
};

template <typename TEndian, typename T, typename TIter>
T read(std::size_t size, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;

    static_assert(std::is_integral<ValueType>::value, "T must be integral type");

    typedef typename std::make_unsigned<ValueType>::type UnsignedType;
    auto value = ReadUnsignedFuncWrapper<TEndian>::template read<UnsignedType>(size, iter);
    return static_cast<T>(static_cast<ValueType>(value));
}

template <typename TEndian, typename T, typename TIter>
T readRandomAccess(std::size_t size, TIter& iter)
{
    typedef typename std::decay<T>::type ValueType;

    static_assert(std::is_integral<ValueType>::value, "T must be integral type");
    static_assert(
        std::is_same<
            typename std::iterator_traits<TIter>::iterator_category,
            std::random_access_iterator_tag
        >::value,
        "TIter must be random access iterator");

    typedef ByteType<TIter> ByteType;
    typedef typename std::make_unsigned<ByteType>::type UnsignedByteType;

    auto startPtr = reinterpret_cast<const UnsignedByteType*>(&(*iter));
    auto endPtr = startPtr;
    auto value = details::read<TEndian, ValueType>(size, endPtr);
    iter += (endPtr - startPtr);
    return static_cast<T>(static_cast<ValueType>(value));
}

template <typename TEndian, bool TIsPointerToUnsignedConst>
struct ReadRandomAccessHelper;

template <typename TEndian>
struct ReadRandomAccessHelper<TEndian, false>
{
    template <typename T, typename TIter>
    static T read(std::size_t size, TIter& iter)
    {
        return readRandomAccess<TEndian, T>(size, iter);
    }
};

template <typename TEndian>
struct ReadRandomAccessHelper<TEndian, true>
{
    template <typename T, typename TIter>
    static T read(std::size_t size, TIter& iter)
    {
        return details::read<TEndian, T>(size, iter);
    }
};

template <typename TEndian, bool TIsRandomAccess>
struct ReadHelper;

template <typename TEndian>
struct ReadHelper<TEndian, false>
{
    template <typename T, typename TIter>
    static T read(std::size_t size, TIter& iter)
    {
        return details::read<TEndian, T>(size, iter);
    }
};

template <typename TEndian>
struct ReadHelper<TEndian, true>
{
    template <typename T, typename TIter>
    static T read(std::size_t size, TIter& iter)
    {
        typedef ByteType<TIter> ByteType;
        static const bool IsPointerToUnsignedConst =
            std::is_pointer<TIter>::value &&
            std::is_const<ByteType>::value &&
            std::is_unsigned<ByteType>::value;
        return ReadRandomAccessHelper<TEndian, IsPointerToUnsignedConst>::template read<T>(size, iter);
    }
};

template <template <typename, bool> class THelper>
struct Writer
{
    template <typename TEndian, std::size_t TSize, typename T, typename TIter>
    static void write(T value, TIter& iter)
    {
        typedef typename std::decay<T>::type ValueType;
        typedef details::OptimisedValueType<ValueType> OptimisedValueType;

        static_assert(TSize <= sizeof(ValueType), "Precondition failure");
        static const bool IsRandomAccess =
            std::is_same<
                typename std::iterator_traits<TIter>::iterator_category,
                std::random_access_iterator_tag
            >::value;
        THelper<TEndian, IsRandomAccess>::write(
                            static_cast<OptimisedValueType>(value), TSize, iter);
    }
};

template <template <typename, bool> class THelper>
struct Reader
{
    template <typename TEndian, typename T, std::size_t TSize, typename TIter>
    static T read(TIter& iter)
    {
        typedef typename std::decay<T>::type ValueType;
        typedef details::OptimisedValueType<ValueType> OptimisedValueType;
        typedef details::ByteType<TIter> ByteType;

        static_assert(TSize <= sizeof(ValueType), "Precondition failure");
        static const bool IsRandomAccess =
            std::is_same<
                typename std::iterator_traits<TIter>::iterator_category,
                std::random_access_iterator_tag
            >::value;
        auto retval =
            static_cast<ValueType>(
                THelper<TEndian, IsRandomAccess>::template read<OptimisedValueType>(TSize, iter));

        if (std::is_signed<ValueType>::value) {
            retval = details::SignExt<decltype(retval), TSize, ByteType>::value(retval);
        }
        return static_cast<T>(retval);
    }
};

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
    details::Writer<details::WriteHelper>::template write<traits::endian::Big, TSize>(value, iter);
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
    return details::Reader<details::ReadHelper>::template read<traits::endian::Big, T, TSize>(iter);
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
    details::Writer<details::WriteHelper>::template write<traits::endian::Little, TSize>(value, iter);
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
    return details::Reader<details::ReadHelper>::template read<traits::endian::Little, T, TSize>(iter);
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
