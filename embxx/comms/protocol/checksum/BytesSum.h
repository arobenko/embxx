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

namespace embxx
{

namespace comms
{

namespace protocol
{

namespace checksum
{

/// @ingroup comms
/// @brief Basic all bytes summary checksum calculator.
/// @details This class summarises all the bytes in the data sequence and
///          returns the result as a checksum value.
/// @tparam TTraits A traits class that must define
///         @li ChecksumLen static integral constant specifying length of
///             checksum field in bytes.
///         @li ChecksumBase static integral constant specifying initial value
///             value of the summary counter.
/// @headerfile embxx/comms/protocol/checksum/BytesSum.h
template <typename TTraits>
class BytesSum
{
public:
    /// @brief Traits
    typedef TTraits Traits;

    /// @brief Length of the message checksum field. Originally defined in traits.
    static const std::size_t ChecksumLen = Traits::ChecksumLen;

    /// @brief Type of the checksum value
    typedef typename util::SizeToType<ChecksumLen>::Type ChecksumType;

    /// @brief Checksum base value
    static const ChecksumType ChecksumBase =
        static_cast<ChecksumType>(Traits::ChecksumBase);

    /// @brief Checksum calculation function.
    /// @tparam TIter Type of input iterator
    /// @param[in, out] iter Input iterator
    /// @param[in] size Size of the data in the buffer
    /// @return Checksum value
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "size" times;
    /// @post The iterator will be advanced by the number of bytes was actually
    ///       read. In case of an error, distance between original position and
    ///       advanced will pinpoint the location of the error.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    template <typename TIter>
    static ChecksumType calc(TIter& iter, std::size_t size)
    {
        ChecksumType checksum = ChecksumBase;
        for (auto idx = 0U; idx < size; ++idx) {
            typedef
                typename std::make_unsigned<
                    typename std::decay<decltype(*iter)>::type
                >::type ByteType;

            checksum += static_cast<ChecksumType>(static_cast<ByteType>(*iter));
            ++iter;
        }

        return checksum;
    }
};

}  // namespace checksum

}  // namespace protocol

}  // namespace comms

}  // namespace embxx




