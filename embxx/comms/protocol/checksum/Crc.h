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

/// @file embxx/comms/protocol/checksum/Crc.h
/// Basic CRC checksum calculator.

#pragma once

#include "boost/crc.hpp"

#include "embxx/util/SizeToType.h"
#include "embxx/util/Assert.h"
#include "embxx/io/access.h"

namespace embxx
{

namespace comms
{

namespace protocol
{

namespace checksum
{

namespace crc_details
{

/// @cond DOCUCMENT_CRC_POLYNOMIAL
template <typename TChecksumType>
struct CrcPolynomial
{
};

template <>
struct CrcPolynomial<std::uint8_t>
{
    static const std::uint8_t Value = 0x07;
};

template <>
struct CrcPolynomial<std::uint16_t>
{
    static const std::uint16_t Value = 0x1021;
};

template <>
struct CrcPolynomial<std::uint32_t>
{
    static const std::uint32_t Value = 0x04C11DB7;
};

template <>
struct CrcPolynomial<std::uint64_t>
{
    static const std::uint64_t Value = 0x000000000000001B;
};

/// @endcond

}

/// @ingroup comms
/// @brief Basic CRC checksum calculator.
/// @details This class uses boost::crc module for calculation of CRC.
///          The truncated polynominals are taken from the table on
///          the wikipedia website (http://en.wikipedia.org/wiki/Cyclic_redundancy_check)
///          and as following:
///          @li CRC-8:  0x07
///          @li CRC-16: 0x1021
///          @li CRC-32: 0x04C11DB7
///          @li CRC-64: 0x000000000000001B
///
///          All other parameters are default ones.
/// @tparam TTraits A traits class that must define
///         @li ChecksumLen static integral constant specifying length of
///             checksum field in bytes.
/// @headerfile embxx/comms/protocol/checksum/Crc.h
template <typename TTraits>
class CrcBasic
{
public:

    /// Traits
    typedef TTraits Traits;

    /// Length of the message checksum field. Originally defined in traits.
    static const std::size_t ChecksumLen = Traits::ChecksumLen;

    /// Type of the checksum value
    typedef typename util::SizeToType<ChecksumLen>::Type ChecksumType;

    /// @brief CRC calculation function.
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
        static const std::size_t Bits = ChecksumLen * 8;
        boost::crc_optimal<
            Bits,
            crc_details::CrcPolynomial<ChecksumType>::Value> crc;
        for (auto count = 0U; count < size; ++count) {
            auto byte = embxx::io::readBig<unsigned char>(iter);
            crc.process_byte(static_cast<unsigned char>(byte));
        }
        auto checksum = crc.checksum();
        return static_cast<ChecksumType>(checksum);
    }
};

}  // namespace checksum

}  // namespace protocol

}  // namespace comms

}  // namespace embxx
