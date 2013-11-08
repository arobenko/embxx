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

/// @file embxx/io/std_streambuf_access.h
/// Simple io module.
/// Provides an ability to put and get integral types into and from
/// steam buffer.

#pragma once

#include <cstddef>
#include <cstdint>
#include <streambuf>
#include <type_traits>

#include "traits.h"

namespace embxx
{

namespace io
{
/// @addtogroup io
/// @{

/// @brief Put integral value into stream buffer with MSB first (Big endian).
/// @tparam T Type of the value. Must be integral type.
/// @param[in] value Value to put into stream buffer.
/// @param[in, out] buf Stream buffer.
/// @return actual number of bytes written to the stream buffer.
/// @pre Stream buffer has enough space to accumulate new value.
///      If not, the behaviour depends on actual streambuf used.
/// @post The internal pointer of the stream buffer is advanced to allow
///       next value being appended.
/// @exception Throws only if used streambuf throws.
/// @note Thread safety: same as thread safety of the used stream buffer.
/// @note Exception guarantee: Basic
template <typename T>
std::size_t putBig(T value, std::streambuf& buf);

/// @brief Put part of integral value into stream buffer with MSB first (Big endian).
/// @tparam TSize Size (in number of bytes) to put to streambuf.
/// @tparam T Type of the value. Must be integral type.
/// @param[in] value Value to put into stream buffer.
/// @param[in, out] buf Stream buffer.
/// @return actual number of bytes written to the stream buffer.
/// @pre Stream buffer has enough space to accumulate new value.
///      If not, the behaviour depends on actual streambuf used.
/// @pre (TSize <= sizeof(T))
/// @post The internal pointer of the stream buffer is advanced to allow
///       next value being appended.
/// @exception Throws only if used streambuf throws.
/// @note Thread safety: same as thread safety of the used stream buffer.
/// @note Exception guarantee: Basic
template <std::size_t TSize, typename T>
std::size_t putBig(T value, std::streambuf& buf);

/// @brief Get integral value from stream buffer with MSB first (Big endian).
/// @tparam T Type of the value. Must be integral type.
/// @param[in, out] buf Stream buffer.
/// @return Value read from buffer.
/// @pre Stream buffer has requested number of bytes available.
///      If not, the behaviour depends on actual streambuf used.
/// @post The internal pointer of the stream buffer is modified to
///       consume the value just read.
/// @exception Throws only if used streambuf throws.
/// @note Thread safety: same as thread safety of the used stream buffer.
/// @note Exception guarantee: Basic
template <typename T>
T getBig(std::streambuf& buf);

/// @brief Get part of integral value from stream buffer with MSB first (Big endian).
/// @tparam T Type of the value. Must be integral type.
/// @tparam TSize Size (in number of bytes) to get from streambuf.
/// @param[in, out] buf Stream buffer.
/// @return Value read from buffer.
/// @pre Stream buffer has requested number of bytes available.
///      If not, the behaviour depends on actual streambuf used.
/// @pre (TSize <= sizeof(T))
/// @post The internal pointer of the stream buffer is modified to
///       consume the value just read.
/// @exception Throws only if used streambuf throws.
/// @note Thread safety: same as thread safety of the used stream buffer.
/// @note Exception guarantee: Basic
template <typename T, std::size_t TSize>
T getBig(std::streambuf& buf);

/// @brief Put integral value into stream buffer with LSB first (Little endian).
/// @tparam T Type of the value. Must be integral type.
/// @param[in] value Value to put into stream buffer.
/// @param[in, out] buf Stream buffer.
/// @return actual number of bytes written to the stream buffer.
/// @pre Stream buffer has enough space to accumulate new value.
///      If not, the behaviour depends on actual streambuf used.
/// @post The internal pointer of the stream buffer is advanced to allow
///       next value being appended.
/// @exception Throws only if used streambuf throws.
/// @note Thread safety: same as thread safety of the used stream buffer.
/// @note Exception guarantee: Basic
template <typename T>
std::size_t putLittle(T value, std::streambuf& buf);

/// @brief Put part of integral value into stream buffer with LSB first (Little endian).
/// @tparam TSize Size (in number of bytes) to put to streambuf.
/// @tparam T Type of the value. Must be integral type.
/// @param[in] value Value to put into stream buffer.
/// @param[in, out] buf Stream buffer.
/// @return actual number of bytes written to the stream buffer.
/// @pre Stream buffer has enough space to accumulate new value.
///      If not, the behaviour depends on actual streambuf used.
/// @pre (TSize <= sizeof(T))
/// @post The internal pointer of the stream buffer is advanced to allow
///       next value being appended.
/// @exception Throws only if used streambuf throws.
/// @note Thread safety: same as thread safety of the used stream buffer.
/// @note Exception guarantee: Basic
template <std::size_t TSize, typename T>
std::size_t putLittle(T value, std::streambuf& buf);

/// @brief Get integral value from stream buffer with LSB first (Little endian).
/// @tparam T Type of the value. Must be integral type.
/// @param[in, out] buf Stream buffer.
/// @return Value read from buffer.
/// @pre Stream buffer has requested number of bytes available.
///      If not, the behaviour depends on actual streambuf used.
/// @post The internal pointer of the stream buffer is modified to
///       consume the value just read
/// @exception Throws only if used streambuf throws.
/// @note Thread safety: same as thread safety of the used stream buffer.
/// @note Exception guarantee: Basic
template <typename T>
T getLittle(std::streambuf& buf);

/// @brief Get part of integral value from stream buffer with LSB first (Little endian).
/// @tparam T Type of the value. Must be integral type.
/// @tparam TSize Size (in number of bytes) to get from streambuf.
/// @param[in, out] buf Stream buffer.
/// @return Value read from buffer.
/// @pre Stream buffer has requested number of bytes available.
///      If not, the behaviour depends on actual streambuf used.
/// @pre (TSize <= sizeof(T))
/// @post The internal pointer of the stream buffer is modified to
///       consume the value just read.
/// @exception Throws only if used streambuf throws.
/// @note Thread safety: same as thread safety of the used stream buffer.
/// @note Exception guarantee: Basic
template <typename T, std::size_t TSize>
T getLittle(std::streambuf& buf);

/// Same as putBig<T>(value, buf)
template <typename T>
std::size_t putData(
    T value,
    std::streambuf& buf,
    const traits::endian::Big& endian);

/// Same as putBig<TSize, T>(value, buf)
template <std::size_t TSize, typename T>
std::size_t putData(
    T value,
    std::streambuf& buf,
    const traits::endian::Big& endian);

/// Same as putLittle<T>(value, buf)
template <typename T>
std::size_t putData(
    T value,
    std::streambuf& buf,
    const traits::endian::Little& endian);

/// Same as putLittle<TSize, T>(value, buf)
template <std::size_t TSize, typename T>
std::size_t putData(
    T value,
    std::streambuf& buf,
    const traits::endian::Little& endian);

/// Same as getBig<T>(buf)
template <typename T>
T getData(std::streambuf& buf, const traits::endian::Big& endian);

/// Same as getBig<TSize, T>(buf)
template <typename T, std::size_t TSize>
T getData(std::streambuf& buf, const traits::endian::Big& endian);

/// Same as getLittle<T>(buf)
template <typename T>
T getData(std::streambuf& buf, const traits::endian::Little& endian);

/// Same as getLittle<TSize, T>(buf)
template <typename T, std::size_t TSize>
T getData(std::streambuf& buf, const traits::endian::Little& endian);

/// @}

// Implementation part

namespace details
{

template <typename T, std::size_t TSize>
class SignExt
{
public:

    static T value(T value)
    {
        T mask = (static_cast<T>(1) << ((TSize * 8) - 1));
        if (value & mask) {
            return value | (~((mask << 1) - 1));
        }
        return value;
    }
};

template <>
class SignExt<std::int8_t, sizeof(std::int8_t)>
{
public:
    static std::int8_t value(std::int8_t value)
    {
        return value;
    }
};

template <>
class SignExt<std::int16_t, sizeof(std::int16_t)>
{
public:
    static std::int16_t value(std::int16_t value)
    {
        return value;
    }
};

template <>
class SignExt<std::int32_t, sizeof(std::int32_t)>
{
public:
    static std::int32_t value(std::int32_t value)
    {
        return value;
    }
};

template <>
class SignExt<std::int64_t, sizeof(std::int64_t)>
{
public:
    static std::int64_t value(std::int64_t value)
    {
        return value;
    }
};

template <typename T>
std::size_t putBig(T value, std::size_t size, std::streambuf& buf)
{
    std::size_t remainingSize = size;
    typedef typename std::make_unsigned<T>::type UnsignedType;
    UnsignedType unsignedValue = static_cast<UnsignedType>(value);
    while (remainingSize > 0) {
        std::size_t remaingShift = ((remainingSize - 1) * 8);
        char byte = static_cast<char>(unsignedValue >> remaingShift);
        auto ch = buf.sputc(byte);
        if (ch == std::streambuf::traits_type::eof()) {
            break;
        }
        --remainingSize;
    }
    return size - remainingSize;
}

template <typename T>
T getBig(std::size_t size, std::streambuf& buf)
{
    typedef typename std::make_unsigned<T>::type UnsignedType;
    UnsignedType value = 0;
    std::size_t remainingSize = size;
    while (remainingSize > 0) {
        auto byte = buf.sbumpc();
        value <<= 8;
        value |= static_cast<std::uint8_t>(byte);
        --remainingSize;
    }

    return static_cast<T>(value);
}

template <typename T>
std::size_t putLittle(T value, std::size_t size, std::streambuf& buf)
{
    std::size_t remainingSize = size;
    typedef typename std::make_unsigned<T>::type UnsignedType;
    UnsignedType unsignedValue = static_cast<UnsignedType>(value);
    while (remainingSize > 0) {
        std::size_t remaingShift = ((size - remainingSize) * 8);
        char byte = static_cast<char>(unsignedValue >> remaingShift);
        auto ch = buf.sputc(byte);
        if (ch == std::streambuf::traits_type::eof()) {
            break;
        }
        --remainingSize;
    }
    return size - remainingSize;
}

template <typename T>
T getLittle(std::size_t size, std::streambuf& buf)
{
    typedef typename std::make_unsigned<T>::type UnsignedType;
    UnsignedType value = 0;
    std::size_t remainingSize = size;
    while (remainingSize > 0) {
        auto byte = static_cast<std::uint8_t>(buf.sbumpc());
        value |= static_cast<UnsignedType>(byte) <<
                            ((size - remainingSize) * 8);
        --remainingSize;
    }

    return static_cast<T>(value);
}

}  // namespace details

template <typename T>
std::size_t putBig(T value, std::streambuf& buf)
{
    return putBig<sizeof(T), T>(value, buf);
}

template <std::size_t TSize, typename T>
std::size_t putBig(T value, std::streambuf& buf)
{
    static_assert(TSize <= sizeof(T), "Precondition failure");
    return details::putBig<T>(value, TSize, buf);
}

template <typename T>
T getBig(std::streambuf& buf)
{
    return getBig<T, sizeof(T)>(buf);
}

template <typename T, std::size_t TSize>
T getBig(std::streambuf& buf)
{
    static_assert(TSize <= sizeof(T), "Precondition failure");
    T retval = details::getBig<T>(TSize, buf);
    if (std::is_signed<T>::value) {
        retval = details::SignExt<T, TSize>::value(retval);
    }
    return retval;
}

template <typename T>
std::size_t putLittle(T value, std::streambuf& buf)
{
    return putLittle<sizeof(T), T>(value, buf);
}

template <std::size_t TSize, typename T>
std::size_t putLittle(T value, std::streambuf& buf)
{
    static_assert(TSize <= sizeof(T), "Precondition failure");
    return details::putLittle<T>(value, TSize, buf);
}

template <typename T>
T getLittle(std::streambuf& buf)
{
    return getLittle<T, sizeof(T)>(buf);
}

template <typename T, std::size_t TSize>
T getLittle(std::streambuf& buf)
{
    static_assert(TSize <= sizeof(T), "Precondition failure");
    T retval = details::getLittle<T>(TSize, buf);
    if (std::is_signed<T>::value) {
        retval = details::SignExt<T, TSize>::value(retval);
    }
    return retval;
}

template <typename T>
std::size_t putData(
    T value,
    std::streambuf& buf,
    const traits::endian::Big& endian)
{
    static_cast<void>(endian);
    return putBig<T>(value, buf);
}

template <std::size_t TSize, typename T>
std::size_t putData(
    T value,
    std::streambuf& buf,
    const traits::endian::Big& endian)
{
    static_cast<void>(endian);
    return putBig<TSize, T>(value, buf);
}

template <typename T>
std::size_t putData(
    T value,
    std::streambuf& buf,
    const traits::endian::Little& endian)
{
    static_cast<void>(endian);
    return putLittle<T>(value, buf);
}

template <std::size_t TSize, typename T>
std::size_t putData(
    T value,
    std::streambuf& buf,
    const traits::endian::Little& endian)
{
    static_cast<void>(endian);
    return putLittle<TSize, T>(value, buf);
}

template <typename T>
T getData(std::streambuf& buf, const traits::endian::Big& endian)
{
    static_cast<void>(endian);
    return getBig<T>(buf);
}

template <typename T, std::size_t TSize>
T getData(std::streambuf& buf, const traits::endian::Big& endian)
{
    static_cast<void>(endian);
    return getBig<T, TSize>(buf);
}


template <typename T>
T getData(std::streambuf& buf, const traits::endian::Little& endian)
{
    static_cast<void>(endian);
    return getLittle<T>(buf);
}

template <typename T, std::size_t TSize>
T getData(std::streambuf& buf, const traits::endian::Little& endian)
{
    static_cast<void>(endian);
    return getLittle<T, TSize>(buf);
}


}  // namespace io

}  // namespace embxx
