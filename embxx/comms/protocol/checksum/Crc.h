//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file comms/protocol/checksum/Crc.h
/// Basic CRC checksum calculator.

#pragma once

#include <streambuf>

#include "boost/crc.hpp"

#include "embxx/util/SizeToType.h"
#include "embxx/util/Assert.h"

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
    /// @param[in, out] buf Input stream buffer
    /// @param[in] size Size of the data in the buffer
    /// @return Checksum value
    /// @post The internal (std::ios_base::in) pointer of the stream buffer
    ///       will be advanced by the number of bytes actually read.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    static ChecksumType calc(std::streambuf& buf, std::size_t size)
    {
        static const std::size_t Bits = ChecksumLen * 8;
        boost::crc_optimal<
            Bits,
            crc_details::CrcPolynomial<ChecksumType>::Value> crc;
        for (auto count = 0U; count < size; ++count) {
            typedef std::streambuf::traits_type TraitsType;
            auto byte = buf.sbumpc();
            GASSERT(static_cast<unsigned>(byte) != std::char_traits<TraitsType>::eof());
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
