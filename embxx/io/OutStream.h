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
#include <array>
#include <algorithm>
#include <iterator>

#include "embxx/util/Assert.h"
#include "StreamManip.h"

namespace embxx
{

namespace io
{

/// @addtogroup io
/// @{

/// @brief Output Stream
/// @details This class is similar to std::ostream, but implements only subset
///          of standard stream functionality without usage of dynamic memory
///          allocation, RTTI, or exceptions, which makes it suitable for use
///          in bare metal platforms with limited amount of memory.
/// @tparam TStreamBuf Output stream buffer (embxx::io::OutStreamBuf) type
template <typename TStreamBuf>
class OutStream
{
public:
    /// @brief Type of stream buffer.
    typedef TStreamBuf StreamBuf;

    /// @brief Character type, provided by the stream buffer.
    typedef typename StreamBuf::CharType CharType;

    /// @brief Unsigned character type
    typedef typename std::make_unsigned<CharType>::type UnsignedCharType;

    /// @brief Constructor
    /// @param buf Reference to output stream buffer. The buffer must be destructed
    ///        after the destruction of this stream object.
    explicit OutStream(StreamBuf& buf);

    /// @brief Copy constructor is deleted
    OutStream(OutStream&) = delete;

    /// @brief Destructor is default
    ~OutStream() = default;

    /// @brief Access the stream buffer object.
    StreamBuf& streamBuf();

    /// @brief Const version of streamBuf().
    const StreamBuf& streamBuf() const;

    /// @brief Set fill character.
    /// @param ch Fill character.
    /// @return Previous fill character
    CharType fill(CharType ch);

    /// @brief Returns current fill character.
    CharType fill() const;

    /// @brief Sets the minimum number of characters to generate on numeric
    ///        output operations
    /// @param value New width
    /// @return The field width before the call to the function.
    std::size_t width(std::size_t value);

    /// @brief Get the current minimum number of characters to generate on
    ///        numeric output operations.
    std::size_t width() const;

    /// @brief Flushes the contents of the stream buffer to the device.
    /// @details Calls flush() member function of the output stream buffer.
    void flush();

    /// @brief Insert "C" (0 terminated) string into the stream.
    /// @param str 0-terminated string.
    /// @return *this
    OutStream& operator<<(const CharType* str);

    /// @brief Insert single character into the stream
    /// @param ch Single character
    /// @return *this
    OutStream& operator<<(CharType ch);

    /// @brief Insert unsigned character into the stream.
    /// @details The character is treated as unsigned number, it is converted
    ///          to string representation based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(UnsignedCharType value);

    /// @brief Insert signed short numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(short value);

    /// @brief Insert unsigned short numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(unsigned short value);

    /// @brief Insert signed int numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(int value);

    /// @brief Insert unsigned int numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(unsigned int value);

    /// @brief Insert signed long long numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(long long value);

    /// @brief Insert unsigned long long numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(unsigned long long value);

    /// @brief End of line manipulator.
    /// @details embxx::io::endl is similar to std::endl, it is equivalent to
    ///          @code
    ///          stream << '\n';
    ///          stream.flush();
    ///          @endcode
    /// @param manip Must be embxx::io::endl
    /// @return *this
    OutStream& operator<<(Endl manip);

    /// @brief End of string manipulator.
    /// @details embxx::io::ends is similar to std::ends, it is equivalent to
    ///          @code
    ///          stream << '\0';
    ///          stream.flush();
    ///          @endcode
    /// @param manip Must be embxx::io::ends
    /// @return *this
    OutStream& operator<<(Ends manip);

    /// @brief Numeric base manipulator.
    /// @details The base manipulator contains the following values:
    ///          @li embxx::io::bin - binary format
    ///          @li embxx::io::oct - octal format, similar to std::oct
    ///          @li embxx::io::dec - decimal format, similar to std::dec
    ///          @li embxx::io::hex - hexadecimal format, similar to std::hex
    /// @param manip Base manipulator
    /// @return *this
    OutStream& operator<<(Base manip);

    /// @brief Width of the field manipulator.
    /// @details Use embxx::io::width() function to create the manipulator object.
    OutStream& operator<<(WidthManip manip);

    /// @brief Fill character manipulator.
    /// @details Use embxx::io::fill() function to create the manipulator object.
    template <typename T>
    OutStream& operator<<(FillManip<T> manip);

private:
    template <typename T, typename TPromotedUnsigned = typename std::make_unsigned<T>::type>
    OutStream& signedToStream(T value);

    template <typename T>
    OutStream& unsignedToStream(T value);

    template <typename T>
    void itoaDec(T value, bool neg);

    template <typename T>
    void itoa(T value);

    StreamBuf& buf_;
    Base base_;
    std::size_t width_;
    CharType fill_;
};

/// @}

// Implementation
template <typename TStreamBuf>
OutStream<TStreamBuf>::OutStream(StreamBuf& buf)
    : buf_(buf),
      base_(dec),
      width_(0),
      fill_(static_cast<CharType>(' '))
{
}

template <typename TStreamBuf>
typename OutStream<TStreamBuf>::StreamBuf&
OutStream<TStreamBuf>::streamBuf()
{
    return buf_;
}

template <typename TStreamBuf>
const typename OutStream<TStreamBuf>::StreamBuf&
OutStream<TStreamBuf>::streamBuf() const
{
    return buf_;
}

template <typename TStreamBuf>
typename OutStream<TStreamBuf>::CharType
OutStream<TStreamBuf>::fill(CharType value)
{
    auto cur = fill_;
    fill_ = value;
    return cur;
}

template <typename TStreamBuf>
typename OutStream<TStreamBuf>::CharType
OutStream<TStreamBuf>::fill() const
{
    return fill_;
}

template <typename TStreamBuf>
std::size_t OutStream<TStreamBuf>::width(std::size_t value)
{
    auto cur = width_;
    width_ = value;
    return cur;
}

template <typename TStreamBuf>
std::size_t OutStream<TStreamBuf>::width() const
{
    return width_;
}


template <typename TStreamBuf>
void OutStream<TStreamBuf>::flush()
{
    buf_.flush();
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(const CharType* str)
{
    buf_.pushBack(str);
    return *this;
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(CharType ch)
{
    buf_.pushBack(ch);
    return *this;
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(UnsignedCharType value)
{
    return (*this << static_cast<unsigned>(value));
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(short value)
{
    return signedToStream<short, unsigned>(value);
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(unsigned short value)
{
    return (*this << static_cast<unsigned>(value));
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(int value)
{
    return signedToStream(value);
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(unsigned int value)
{
    return unsignedToStream(value);
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(long long value)
{
    return signedToStream(value);
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(unsigned long long value)
{
    return unsignedToStream(value);
}

template <typename TStreamBuf>
template <typename T, typename TPromotedUnsigned>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::signedToStream(T value)
{
    typedef typename std::make_unsigned<T>::type UnsignedType;
    if ((0 <= value) || (base_ != dec)) {
        return (*this << static_cast<UnsignedType>(value));
    }

    itoaDec(static_cast<TPromotedUnsigned>(static_cast<UnsignedType>(-value)), true);
    return *this;
}

template <typename TStreamBuf>
template <typename T>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::unsignedToStream(T value)
{
    if (base_ == dec) {
        itoaDec(value, false);
    }
    else {
        itoa(value);
    }
    return *this;
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(Endl manip)
{
    static_cast<void>(manip);
    buf_.pushBack('\n');
    flush();
    return *this;
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(Ends manip)
{
    static_cast<void>(manip);
    buf_.pushBack('\0');
    flush();
    return *this;
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(Base manip)
{
    if (Base_NumOfBases <= manip) {
        GASSERT(!"Unexpected manipulator");
        return *this;
    }

    base_ = manip;
    return *this;
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(WidthManip manip)
{
    width(manip.value());
    return *this;
}

template <typename TStreamBuf>
template <typename T>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(FillManip<T> manip)
{
    fill(static_cast<CharType>(manip.value()));
    return *this;
}


template <typename TStreamBuf>
template <typename T>
void OutStream<TStreamBuf>::itoaDec(T value, bool sign)
{
    GASSERT(base_ == dec);
    typedef typename std::make_signed<T>::type SignedType;
    static const std::size_t StrSize = std::numeric_limits<SignedType>::digits10 + 2;
    std::array<CharType, StrSize> tmpBuf;

    auto iter = tmpBuf.begin();
    while (value != 0) {
        static const T Base = 10;
        auto res = value % Base;
        auto ch = static_cast<CharType>(res) + static_cast<CharType>('0');

        GASSERT(iter != tmpBuf.end());
        *iter = ch;
        ++iter;
        value /= Base;
    }

    if (iter == tmpBuf.begin()) {
        *iter = static_cast<CharType>('0');
        ++iter;
        GASSERT(!sign);
    }

    auto strSize =
        static_cast<std::size_t>(std::distance(tmpBuf.begin(), iter));
    if (sign) {
        ++strSize;
    }

    if (strSize < width_) {
        auto fillLen = width_ - strSize;
        std::fill_n(std::back_inserter(buf_), fillLen, fill_);
    }

    if (sign) {
        GASSERT(iter != tmpBuf.end());
        buf_.pushBack(static_cast<CharType>('-'));
    }

    auto revIterOffset = std::distance(iter, tmpBuf.end());
    auto printIter = tmpBuf.rbegin() + revIterOffset;
    std::copy(printIter, tmpBuf.rend(), std::back_inserter(buf_));
}

template <typename TStreamBuf>
template <typename T>
void OutStream<TStreamBuf>::itoa(T value)
{
    GASSERT(base_ < Base_NumOfBases);
    GASSERT(base_ != dec);

    static const std::size_t Shift[Base_NumOfBases] =
    {
        /* bin */ 1,
        /* oct */ 3,
        /* dec */ 0,
        /* hex */ 4
    };
    auto shift = Shift[base_];

    static const std::size_t StrSize = std::numeric_limits<T>::digits;
    std::array<CharType, StrSize> tmpBuf;

    auto iter = tmpBuf.begin();
    while (value != 0) {
        T printValMask = (static_cast<T>(1) << shift) - 1;
        auto maskedValue = value & printValMask;
        CharType ch = 0;
        static const T Base = 10;
        if (maskedValue < Base) {
            ch = static_cast<CharType>(maskedValue) + static_cast<CharType>('0');
        }
        else {
            ch =
                static_cast<CharType>(maskedValue - Base) +
                static_cast<CharType>('a');
        }

        GASSERT(iter != tmpBuf.end());
        *iter = ch;
        ++iter;
        value >>= shift;
    }

    if (iter == tmpBuf.begin()) {
        *iter = static_cast<CharType>('0');
        ++iter;
    }

    auto strSize =
        static_cast<std::size_t>(std::distance(tmpBuf.begin(), iter));
    if (strSize < width_) {
        auto fillLen = width_ - strSize;
        std::fill_n(std::back_inserter(buf_), fillLen, fill_);
    }

    auto revIterOffset = std::distance(iter, tmpBuf.end());
    auto printIter = tmpBuf.rbegin() + revIterOffset;
    std::copy(printIter, tmpBuf.rend(), std::back_inserter(buf_));
}

}  // namespace io

}  // namespace embxx
