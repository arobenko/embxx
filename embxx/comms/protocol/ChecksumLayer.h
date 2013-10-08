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

/// @file embxx/comms/protocol/ChecksumLayer.h
/// This file contains checksum layer for to comms module.

#pragma once

#include "embxx/util/Assert.h"
#include "embxx/util/SizeToType.h"
#include "embxx/comms/traits.h"
#include "ProtocolLayer.h"

namespace embxx
{

namespace comms
{

namespace protocol
{

/// @ingroup comms
/// @brief Protocol layer that uses uses a checksum field to verify all the
///        the data written by the next layer(s) is valid.
/// @details This layer is a mid level layer, expects other mid level layer or
///          MsgDataLayer to be its next one.
/// @tparam TTraits A traits class that must define:
///         @li Endianness type. Either embxx::comms::traits::endian::Big or
///             embxx::comms::traits::endian::Little
///         @li ChecksumVerification type. Either
///             embxx::comms::traits::checksum::VerifyBeforeProcessing or
///             embxx::comms::traits::checksum::VerifyAfterProcessing to indicate
///             the order in which checksum verification is executed when
///             deserialising message.
///         @li ChecksumLen static integral constant of type std::size_t
///             specifying length of checksum field in bytes.
/// @tparam TChecksumCalc Class that defines static
///         "typename embxx::util::SizeToType<ChecksumLen>::Type calc(std::streambuf& buf, std::size_t size)"
///         function which can be used to calculate checksum of the data in
///         the input buffer.
/// @tparam TNextLayer Next layer in the protocol stack
/// @headerfile embxx/comms/protocol/ChecksumLayer.h
template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
class ChecksumLayer : public ProtocolLayer<TTraits, TNextLayer>
{
    typedef ProtocolLayer<TTraits, TNextLayer> Base;
public:

    /// @brief Pointer to message object
    typedef typename Base::MsgPtr MsgPtr;

    /// @brief Base class to all custom messages
    typedef typename Base::MsgBase MsgBase;

    /// @brief Traits
    typedef typename Base::Traits Traits;

    /// @brief Checksum verifiation behaviour
    typedef typename Traits::ChecksumVerification ChecksumVerification;

    /// @brief Length of the message checksum field. Originally defined in traits.
    static const std::size_t ChecksumLen = Traits::ChecksumLen;

    /// @brief Type of the "checksum" field
    typedef typename util::SizeToType<ChecksumLen>::Type ChecksumType;

    /// @brief Checksum calculator
    typedef TChecksumCalc ChecksumCalc;

    /// @brief Constructor
    ChecksumLayer() = default;

    /// @brief Deserialise message from the data in the input stream buffer.
    /// @details The functionality of whether to verify checksum before or
    ///          after forwarding read() request to the next layer is determined
    ///          by ChecksumVerification trait. In case it is defined to be
    ///          embxx::comms::traits::checksum::VerifyBeforeProcessing, then the
    ///          read() function of the next layer won't be called if checksum
    ///          verification fails. If it is defined to be
    ///          embxx::comms::traits::checksum::VerifyAfterProcessing, then the read()
    ///          function of the next layer will be called immediately in attempt
    ///          to create a message object. If the creation is successful, only
    ///          then the validity of the message object will be verified. In
    ///          case the verification fails the message object will be destructed.
    /// @param[in, out] msgPtr Reference to smart pointer that already holds or
    ///                 will hold allocated message object
    /// @param[in, out] buf Input stream buffer
    /// @param[in] size Size of the data in the buffer
    /// @return Error status of the operation.
    /// @post The internal (std::ios_base::in) pointer of the stream buffer
    ///       will be advanced by the number of bytes actually read.
    ///       In case of an error, it will provide an information to the caller
    ///       about the place the error was recognised.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    template <typename TMsgPtr>
    ErrorStatus read(TMsgPtr& msgPtr, std::streambuf& buf, std::size_t size);

    /// @brief Calculate checksum of the data in the stream buffer.
    /// @details The read is executed from current input position of the buffer.
    ///          The internal pointer will NOT be returned to its original
    ///          position.
    /// @param[in, out] buf Input stream buffer
    /// @param[in] size Size of the data in the buffer
    /// @return Value of the checksum
    /// @post The internal (std::ios_base::in) pointer of the stream buffer
    ///       will be advanced by the number of bytes actually read.
    ///       In case of an error, it will provide an information to the caller
    ///       about the place the error was recognised.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    static ChecksumType calcChecksum(
        std::streambuf& buf,
        std::size_t size);

    /// @brief Serialise message into the stream buffer.
    /// @details The function will call write() member function of the
    ///          next layer, calculate a checksum for the written
    ///          data and append this checksum to the already written data.
    /// @param[in] msg Reference to message object
    /// @param[in, out] buf Input/Output stream buffer.
    /// @param[in] size size of the buffer
    /// @return Status of the write operation.
    /// @pre Stream buffer must be both Input and Output buffer, i.e. provide
    ///      both read and write operations.
    /// @post The internal (std::ios_base::out) pointer of the stream buffer
    ///       will be advanced by the number of bytes was actually written.
    ///       In case of an error, it will provide an information to the caller
    ///       about the place the error was recognised.
    /// @post The internal (std::ios_base::in) pointer of the stream buffer
    ///       will be advanced by the number of bytes was written before
    ///       checksum calculation.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    ErrorStatus write(
                const MsgBase& msg,
                std::streambuf& buf,
                std::size_t size) const;

private:

    template <typename TMsgPtr>
    ErrorStatus readInternal(
        TMsgPtr& msgPtr,
        std::streambuf& buf,
        std::size_t size,
        const traits::checksum::VerifyBeforeProcessing& behavour);

    template <typename TMsgPtr>
    ErrorStatus readInternal(
        TMsgPtr& msgPtr,
        std::streambuf& buf,
        std::size_t size,
        const traits::checksum::VerifyAfterProcessing& behavour);


};

// Implementation
template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
template <typename TMsgPtr>
ErrorStatus ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::read(
    TMsgPtr& msgPtr,
    std::streambuf& buf,
    std::size_t size)
{
    static_assert(std::is_base_of<MsgBase, typename std::decay<decltype(*msgPtr)>::type>::value,
        "TMsgBase must be a base class of decltype(*msgPtr)");
    GASSERT(size <= static_cast<decltype(size)>(buf.in_avail()));

    if (size < ChecksumLen) {
        return ErrorStatus::NotEnoughData;
    }

    return readInternal(msgPtr, buf, size, ChecksumVerification());
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
typename ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::ChecksumType
ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::calcChecksum(
        std::streambuf& buf,
        std::size_t size)
{
    return ChecksumCalc::calc(buf, size);
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
ErrorStatus ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::write(
            const MsgBase& msg,
            std::streambuf& buf,
            std::size_t size) const
{
    const auto BufMode = std::ios_base::out;
    auto firstPos = buf.pubseekoff(0, std::ios_base::cur, BufMode);

#ifndef NDEBUG
    auto lastPos = buf.pubseekoff(0, std::ios_base::end, BufMode);
    buf.pubseekpos(firstPos, BufMode);
    auto diff = static_cast<decltype(size)>(lastPos - firstPos);
    GASSERT(size <= diff);
#endif // #ifndef NDEBUG

    auto status = Base::nextLayer().write(msg, buf, size - ChecksumLen);
    if (status != ErrorStatus::Success)
    {
        return status;
    }

    auto curPos = buf.pubseekoff(0, std::ios_base::cur, BufMode);
    auto curSize = static_cast<decltype(size)>(curPos - firstPos);
    GASSERT(curSize <= size);

    if (size < (curSize + ChecksumLen)) {
        return ErrorStatus::BufferOverflow;
    }

    buf.pubseekpos(firstPos, std::ios_base::in);
    auto checksum = calcChecksum(buf, curSize);
    Base::template putData<ChecksumLen>(checksum, buf);
    return ErrorStatus::Success;
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
template <typename TMsgPtr>
ErrorStatus ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::readInternal(
    TMsgPtr& msgPtr,
    std::streambuf& buf,
    std::size_t size,
    const traits::checksum::VerifyBeforeProcessing& behaviour)
{
    static_cast<void>(behaviour);

    const auto BufMode = std::ios_base::in;
    auto firstPos = buf.pubseekoff(0, std::ios_base::cur, BufMode);
    auto calculatedChecksum = calcChecksum(buf, size - ChecksumLen);
    auto expectedChecksum = Base::template getData<ChecksumType, ChecksumLen>(buf);
    if (calculatedChecksum != expectedChecksum) {
        return ErrorStatus::ProtocolError;
    }
    auto lastPos = buf.pubseekoff(0, std::ios_base::cur, BufMode);

    buf.pubseekpos(firstPos, BufMode);
    auto status = Base::nextLayer().read(msgPtr, buf, size - ChecksumLen);
    if (status != ErrorStatus::Success) {
        return ErrorStatus::ProtocolError;
    }
    buf.pubseekpos(lastPos, BufMode);
    return status;
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
template <typename TMsgPtr>
ErrorStatus ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::readInternal(
    TMsgPtr& msgPtr,
    std::streambuf& buf,
    std::size_t size,
    const traits::checksum::VerifyAfterProcessing& behaviour)
{
    static_cast<void>(behaviour);

    const auto BufMode = std::ios_base::in;
    auto firstPos = buf.pubseekoff(0, std::ios_base::cur, BufMode);

    auto status = Base::nextLayer().read(msgPtr, buf, size - ChecksumLen);
    if (status != ErrorStatus::Success) {
        return status;
    }

    auto lastPos = buf.pubseekoff(0, std::ios_base::cur, BufMode);
    auto posDiff = static_cast<std::size_t>(lastPos - firstPos);
    if (size < (posDiff + ChecksumLen)) {
        msgPtr.reset();
        return ErrorStatus::NotEnoughData;
    }

    buf.pubseekpos(firstPos, BufMode);
    auto calculatedChecksum = calcChecksum(buf, posDiff);
    auto expectedChecksum =
                Base::template getData<ChecksumType, ChecksumLen>(buf);
    if (calculatedChecksum != expectedChecksum) {
        msgPtr.reset();
        return ErrorStatus::ProtocolError;
    }

    return ErrorStatus::Success;
}

}  // namespace protocol

}  // namespace comms

}  // namespace embxx
