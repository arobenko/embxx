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

#include "embxx/error/ErrorStatus.h"
#include "embxx/util/Assert.h"
#include "embxx/util/SizeToType.h"
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
/// @headerfile embxx/io/OutStream.h
template <typename TStreamBuf>
class OutStream
{
public:
    /// @brief Type of stream buffer.
    typedef TStreamBuf StreamBuf;

    /// @brief Character type, provided by the stream buffer.
    typedef typename StreamBuf::CharType CharType;

    /// @brief Constructor
    /// @param buf Reference to output stream buffer. The buffer must be destructed
    ///        after the destruction of this stream object.
    explicit OutStream(StreamBuf& buf)
    : buf_(buf),
      base_(dec),
      width_(0),
      fill_(static_cast<CharType>(' '))
    {
    }

    /// @brief Copy constructor is deleted
    OutStream(OutStream&) = delete;

    /// @brief Destructor is default
    ~OutStream() = default;

    /// @brief Access the stream buffer object.
    StreamBuf& streamBuf()
    {
        return buf_;
    }

    /// @brief Const version of streamBuf().
    const StreamBuf& streamBuf() const
    {
        return buf_;
    }

    /// @brief Set fill character.
    /// @param ch Fill character.
    /// @return Previous fill character
    CharType fill(CharType ch)
    {
        auto cur = fill_;
        fill_ = ch;
        return cur;
    }

    /// @brief Returns current fill character.
    CharType fill() const
    {
        return fill_;
    }

    /// @brief Sets the minimum number of characters to generate on numeric
    ///        output operations
    /// @param value New width
    /// @return The field width before the call to the function.
    std::size_t width(std::size_t value)
    {
        auto cur = width_;
        width_ = value;
        return cur;
    }

    /// @brief Get the current minimum number of characters to generate on
    ///        numeric output operations.
    std::size_t width() const
    {
        return width_;
    }

    /// @brief Flushes the contents of the stream buffer to the device.
    /// @details Calls flush() member function of the output stream buffer.
    void flush()
    {
        buf_.flush();
    }

    /// @brief Insert "C" (0 terminated) string into the stream.
    /// @param str 0-terminated string.
    /// @return *this
    OutStream& operator<<(const CharType* str)
    {
        buf_.pushBack(str);
        return *this;
    }

    /// @brief Insert single character into the stream
    /// @param ch Single character
    /// @return *this
    OutStream& operator<<(char ch)
    {
        buf_.pushBack(ch);
        return *this;
    }

    /// @brief Insert 8 bit unsigned numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(std::uint8_t value)
    {
        return (*this << static_cast<unsigned>(value));
    }

    /// @brief Insert 16 bit signed numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(std::int16_t value)
    {
        return signedToStream<std::int16_t, std::uint32_t>(value);
    }

    /// @brief Insert 16 bit unsigned numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(std::uint16_t value)
    {
        return (*this << static_cast<std::uint32_t>(value));
    }

    /// @brief Insert 32 bit signed numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(std::int32_t value)
    {
        return signedToStream(value);
    }

    /// @brief Insert 32 bit unsigned numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(std::uint32_t value)
    {
        return unsignedToStream(value);
    }

    /// @brief Insert 64 bit signed numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(std::int64_t value)
    {
        return signedToStream(value);
    }

    /// @brief Insert 64 bit unsigned numeric value into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param value Numeric value
    /// @return *this
    OutStream& operator<<(std::uint64_t value)
    {
        return unsignedToStream(value);
    }

    /// @brief Insert numeric value of the error code wrapped by ErrorStatusT
    ///        into the stream.
    /// @details The numeric value is converted to string representation
    ///          based on the current base modifier and
    ///          inserted into the stream buffer.
    /// @param es ErrorStatus object
    /// @return *this
    template <typename TCode>
    OutStream& operator<<(const embxx::error::ErrorStatusT<TCode>& es)
    {
        return (*this << es.code());
    }

    /// @brief End of line manipulator.
    /// @details embxx::io::endl is similar to std::endl, it is equivalent to
    ///          @code
    ///          stream << '\n';
    ///          stream.flush();
    ///          @endcode
    /// @param manip Must be embxx::io::endl
    /// @return *this
    OutStream& operator<<(Endl manip)
    {
        static_cast<void>(manip);
        buf_.pushBack('\n');
        flush();
        return *this;
    }

    /// @brief End of string manipulator.
    /// @details embxx::io::ends is similar to std::ends, it is equivalent to
    ///          @code
    ///          stream << '\0';
    ///          stream.flush();
    ///          @endcode
    /// @param manip Must be embxx::io::ends
    /// @return *this
    OutStream& operator<<(Ends manip)
    {
        static_cast<void>(manip);
        buf_.pushBack('\0');
        flush();
        return *this;
    }

    /// @brief Numeric base manipulator.
    /// @details The base manipulator contains the following values:
    ///          @li embxx::io::bin - binary format
    ///          @li embxx::io::oct - octal format, similar to std::oct
    ///          @li embxx::io::dec - decimal format, similar to std::dec
    ///          @li embxx::io::hex - hexadecimal format, similar to std::hex
    /// @param manip Base manipulator
    /// @return *this
    OutStream& operator<<(Base manip)
    {
        if (Base_NumOfBases <= manip) {
            GASSERT(!"Unexpected manipulator");
            return *this;
        }

        base_ = manip;
        return *this;
    }

    /// @brief Width of the field manipulator.
    /// @details Use embxx::io::setw() function to create the manipulator object.
    ///          @code
    ///          stream << std::io::setw(8) << value;
    ///          @endcode
    OutStream& operator<<(details::WidthManip manip)
    {
        width(manip.value());
        return *this;
    }

    /// @brief Fill character manipulator.
    /// @details Use embxx::io::setfill() function to create the manipulator object.
    ///          @code
    ///          stream << std::io::setfill('0') << value;
    ///          @endcode
    template <typename T>
    OutStream& operator<<(details::FillManip<T> manip)
    {
        fill(static_cast<CharType>(manip.value()));
        return *this;
    }

    /// @brief Generic print operator.
    /// @details If none of other operator<<() member functions were chosen,
    ///          tries to make a best guess of which one to use.
    template <typename T>
    OutStream& operator<<(T value)
    {
        typedef typename
            std::conditional<
                std::is_enum<T>::value,
                EnumTypeTag,
                typename std::conditional<
                    std::is_integral<T>::value, IntegralTypeTag, NonIntegralTypeTag
                >::type
            >::type Tag;

        printInternal(value, Tag());
        return *this;
    }

private:
    struct IntegralTypeTag {};
    struct EnumTypeTag {};
    struct NonIntegralTypeTag {};

    template <typename T, typename TPromotedUnsigned = typename std::make_unsigned<T>::type>
    OutStream& signedToStream(T value)
    {
        typedef typename std::make_unsigned<T>::type UnsignedType;
        if ((0 <= value) || (base_ != dec)) {
            return (*this << static_cast<UnsignedType>(value));
        }

        itoaDec(static_cast<TPromotedUnsigned>(static_cast<UnsignedType>(-value)), true);
        return *this;
    }

    template <typename T>
    OutStream& unsignedToStream(T value)
    {
        if (base_ == dec) {
            itoaDec(value, false);
        }
        else {
            itoa(value);
        }
        return *this;
    }

    template <typename T>
    void itoaDec(T value, bool sign)
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

    template <typename T>
    void itoa(T value)
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

    template <typename T>
    void printInternal(T value, IntegralTypeTag)
    {
        typedef typename
            embxx::util::SizeToType<
                sizeof(value),
                std::is_signed<T>::value
            >::Type CastedType;
        *this << static_cast<CastedType>(value);
    }

    template <typename T>
    void printInternal(T value, EnumTypeTag)
    {
        typedef typename std::underlying_type<T>::type UnderlyingType;
        printInternal(static_cast<UnderlyingType>(value), IntegralTypeTag());
    }

    template <typename T>
    void printInternal(T value, NonIntegralTypeTag)
    {
        static_cast<void>(value);
        static_assert(std::is_integral<T>::value,
            "Currently output of non-integral type is not supported");
    }

    StreamBuf& buf_;
    Base base_;
    std::size_t width_;
    CharType fill_;
};

/// @}

}  // namespace io

}  // namespace embxx
