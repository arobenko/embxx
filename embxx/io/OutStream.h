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

template <typename TStreamBuf>
class OutStream
{
public:
    typedef TStreamBuf StreamBuf;

    typedef typename StreamBuf::CharType CharType;

    typedef typename std::make_unsigned<CharType>::type UnsignedCharType;

    explicit OutStream(StreamBuf& buf);

    ~OutStream() = default;

    StreamBuf& streamBuf();

    const StreamBuf& streamBuf() const;

    void fill(CharType ch);

    void width(std::size_t value);

    void flush();

    OutStream& operator<<(const CharType* str);

    OutStream& operator<<(CharType ch);

    OutStream& operator<<(UnsignedCharType value);

    OutStream& operator<<(std::int16_t value);

    OutStream& operator<<(std::uint16_t value);

    OutStream& operator<<(std::int32_t value);

    OutStream& operator<<(std::uint32_t value);

    OutStream& operator<<(std::int64_t value);

    OutStream& operator<<(std::uint64_t value);

    OutStream& operator<<(Endl manip);

    OutStream& operator<<(Ends manip);

    OutStream& operator<<(Base manip);

private:
    template <typename T>
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
void OutStream<TStreamBuf>::fill(CharType value)
{
    fill_ = value;
}

template <typename TStreamBuf>
void OutStream<TStreamBuf>::width(std::size_t value)
{
    width_ = value;
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
    return (*this << static_cast<std::uint32_t>(value));
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(std::int16_t value)
{
    return (*this << static_cast<std::int32_t>(value));
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(std::uint16_t value)
{
    return (*this << static_cast<std::uint32_t>(value));
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(std::int32_t value)
{
    return signedToStream(value);
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(std::uint32_t value)
{
    return unsignedToStream(value);
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(std::int64_t value)
{
    return signedToStream(value);
}

template <typename TStreamBuf>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::operator<<(std::uint64_t value)
{
    return unsignedToStream(value);
}

template <typename TStreamBuf>
template <typename T>
OutStream<TStreamBuf>&
OutStream<TStreamBuf>::signedToStream(T value)
{
    if ((0 <= value) || (base_ != dec)) {
        return (*this << static_cast<std::uint64_t>(value));
    }

    itoaDec(static_cast<std::uint64_t>(-value), true);
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

    auto strSize = std::distance(tmpBuf.begin(), iter);
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

    auto strSize = std::distance(tmpBuf.begin(), iter);
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
