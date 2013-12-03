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
///         @code
///         template <typename TIter>
///         typename embxx::util::SizeToType<ChecksumLen>::Type calc(TIter& iter, std::size_t size);
///         @endcode
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

    /// @brief Type of read iterator
    typedef typename Base::ReadIterator ReadIterator;

    /// @brief Type of write iterator
    typedef typename Base::WriteIterator WriteIterator;


    /// @brief Constructor
    template <typename... TArgs>
    explicit ChecksumLayer(TArgs&&... args);

    /// @brief Copy constructor is default
    ChecksumLayer(const ChecksumLayer&) = default;

    /// @brief Move constructor is default
    ChecksumLayer(ChecksumLayer&&) = default;

    /// @brief Destructor is default
    ~ChecksumLayer() = default;

    /// @brief Copy assignment operator is default
    ChecksumLayer& operator=(const ChecksumLayer&) = default;

    /// @brief Move assignment operator is default
    ChecksumLayer& operator=(ChecksumLayer&&) = default;

    /// @brief Deserialise message from the data in the input data sequence.
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
    /// @param[in, out] iter Input iterator
    /// @param[in] size Size of the data in the buffer
    /// @param[out] missingSize If not nullptr and return value is
    ///             embxx::comms::ErrorStatus::NotEnoughData it will contain
    ///             minimal missing data length required for the successful
    ///             read attempt.
    /// @return Error status of the operation.
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "size" times;
    /// @post The iterator will be advanced by the number of bytes was actually
    ///       read. In case of an error, distance between original position and
    ///       advanced will pinpoint the location of the error.
    /// @post missingSize output value is updated if and only if function
    ///       returns embxx::comms::ErrorStatus::NotEnoughData.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    template <typename TMsgPtr>
    ErrorStatus read(
        TMsgPtr& msgPtr,
        ReadIterator& iter,
        std::size_t size,
        std::size_t* missingSize = nullptr);

    /// @brief Calculate checksum of the data in the input data sequence.
    /// @details The read is executed from current position of the iterator.
    /// @tparam TIter Type of iterator.
    /// @param[in, out] iter iterator.
    /// @param[in] size Size of the data in the buffer
    /// @return Value of the checksum
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "size" times;
    /// @post The iterator will be advanced by the number of bytes was actually
    ///       read. In case of an error, distance between original position and
    ///       advanced will pinpoint the location of the error.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    template <typename TIter>
    static ChecksumType calcChecksum(
        TIter& iter,
        std::size_t size);

    /// @brief Serialise message into the output data sequence.
    /// @details The function will call write() member function of the
    ///          next layer, if possible calculate a checksum for the written
    ///          data and append this checksum to the already written data. If
    ///          it is not possible to return iterator to its original position
    ///          in order to start checksum calculation (for example
    ///          std::back_insert_iterator was used) embxx::comms::ErrorStatus::UpdateRequired
    ///          will be returned. In this case it is needed to call update()
    ///          member function to finalise the write operation.
    /// @param[in] msg Reference to message object
    /// @param[in, out] iter Output iterator.
    /// @param[in] size size of the buffer
    /// @return Status of the write operation.
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "size" times;
    /// @post The iterator will be advanced by the number of bytes was actually
    ///       written. In case of an error, distance between original position
    ///       and advanced will pinpoint the location of the error.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    ErrorStatus write(
                const MsgBase& msg,
                WriteIterator& iter,
                std::size_t size) const;

    /// @brief Update the recently written output data sequence.
    /// @copydetails MsgIdLayer::update
    template <typename TUpdateIter>
    ErrorStatus update(
        TUpdateIter& iter,
        std::size_t size) const;

    /// @brief Returns minimal protocol stack length required to serialise
    ///        empty message.
    /// @details Adds "ChecksumLen" to the result of nextLayer().length().
    constexpr std::size_t length() const;

    /// @brief Returns message serialisation length including protocol stack
    ///        overhead.
    /// @details Adds "ChecksumLen" to the result of nextLayer().length(msg).
    std::size_t length(const MsgBase& msg) const;

private:

    template <typename TMsgPtr>
    ErrorStatus readInternal(
        TMsgPtr& msgPtr,
        ReadIterator& iter,
        std::size_t size,
        std::size_t* missingSize,
        const traits::checksum::VerifyBeforeProcessing& behavour);

    template <typename TMsgPtr>
    ErrorStatus readInternal(
        TMsgPtr& msgPtr,
        ReadIterator& iter,
        std::size_t size,
        std::size_t* missingSize,
        const traits::checksum::VerifyAfterProcessing& behavour);

    ErrorStatus writeInternal(
        const MsgBase& msg,
        WriteIterator& iter,
        std::size_t size,
        const std::random_access_iterator_tag& tag) const;

    ErrorStatus writeInternal(
        const MsgBase& msg,
        WriteIterator& iter,
        std::size_t size,
        const std::output_iterator_tag& tag) const;
};

// Implementation
template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
template <typename... TArgs>
ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::ChecksumLayer(
    TArgs&&... args)
    : Base(std::forward<TArgs>(args)...)
{
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
template <typename TMsgPtr>
ErrorStatus ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::read(
    TMsgPtr& msgPtr,
    ReadIterator& iter,
    std::size_t size,
    std::size_t* missingSize)
{
    static_assert(std::is_base_of<MsgBase, typename std::decay<decltype(*msgPtr)>::type>::value,
        "TMsgBase must be a base class of decltype(*msgPtr)");

    if (size < ChecksumLen) {
        if (missingSize != nullptr) {
            *missingSize = length() - size;
        }
        return ErrorStatus::NotEnoughData;
    }

    return readInternal(msgPtr, iter, size, missingSize, ChecksumVerification());
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
template <typename TIter>
typename ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::ChecksumType
ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::calcChecksum(
        TIter& iter,
        std::size_t size)
{
    return ChecksumCalc::calc(iter, size);
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
ErrorStatus ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::write(
            const MsgBase& msg,
            WriteIterator& iter,
            std::size_t size) const
{
    typedef typename std::iterator_traits<WriteIterator>::iterator_category IterType;
    return writeInternal(msg, iter, size, IterType());
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
template <typename TUpdateIter>
ErrorStatus ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::update(
    TUpdateIter& iter,
    std::size_t size) const
{
    TUpdateIter firstPosIter(iter);
    auto errorStatus = Base::nextLayer().update(iter, size - ChecksumLen);
    if (errorStatus == ErrorStatus::Success) {
        auto checksum = calcChecksum(firstPosIter, size - ChecksumLen);
        GASSERT(iter == firstPosIter);
        Base::template writeData<ChecksumLen>(checksum, iter);
    }
    return errorStatus;
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
constexpr
std::size_t ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::length() const
{
    return ChecksumLen + Base::nextLayer().length();
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
std::size_t ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::length(
    const MsgBase& msg) const
{
    return ChecksumLen + Base::nextLayer().length(msg);
}


template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
template <typename TMsgPtr>
ErrorStatus ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::readInternal(
    TMsgPtr& msgPtr,
    ReadIterator& iter,
    std::size_t size,
    std::size_t* missingSize,
    const traits::checksum::VerifyBeforeProcessing& behaviour)
{
    static_cast<void>(behaviour);

    auto calculatedChecksum = calcChecksum(iter, size - ChecksumLen);
    auto expectedChecksum =
        Base::template readData<ChecksumType, ChecksumLen>(iter);
    if (calculatedChecksum != expectedChecksum) {
        return ErrorStatus::ProtocolError;
    }

    auto advanceSize = -(static_cast<int>(size));
    std::advance(iter, advanceSize);

    auto status = Base::nextLayer().read(msgPtr, iter, size - ChecksumLen, missingSize);
    if (status != ErrorStatus::Success) {
        return ErrorStatus::ProtocolError;
    }

    std::advance(iter, ChecksumLen);
    return status;
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
template <typename TMsgPtr>
ErrorStatus ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::readInternal(
    TMsgPtr& msgPtr,
    ReadIterator& iter,
    std::size_t size,
    std::size_t* missingSize,
    const traits::checksum::VerifyAfterProcessing& behaviour)
{
    static_cast<void>(behaviour);

    ReadIterator firstPosIter(iter);
    auto status = Base::nextLayer().read(msgPtr, iter, size, missingSize);
    if (status != ErrorStatus::Success) {
        if ((status == ErrorStatus::NotEnoughData) &&
            (missingSize != nullptr)) {
            *missingSize += ChecksumLen;
        }
        return status;
    }

    auto posDiff = static_cast<std::size_t>(std::distance(firstPosIter, iter));
    GASSERT(posDiff <= size);
    if (size < (posDiff + ChecksumLen)) {
        if (missingSize != nullptr) {
            *missingSize = (posDiff + ChecksumLen) - size;
        }
        msgPtr.reset();
        return ErrorStatus::NotEnoughData;
    }

    auto calculatedChecksum = calcChecksum(firstPosIter, posDiff);
    auto expectedChecksum =
                Base::template readData<ChecksumType, ChecksumLen>(iter);
    if (calculatedChecksum != expectedChecksum) {
        msgPtr.reset();
        return ErrorStatus::ProtocolError;
    }

    return ErrorStatus::Success;
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
ErrorStatus ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::writeInternal(
    const MsgBase& msg,
    WriteIterator& iter,
    std::size_t size,
    const std::random_access_iterator_tag& tag) const
{
    static_cast<void>(tag);

    WriteIterator firstPosIter(iter);

    auto status = Base::nextLayer().write(msg, iter, size - ChecksumLen);
    if (status != ErrorStatus::Success)
    {
        return status;
    }

    auto curSize = static_cast<decltype(size)>(iter - firstPosIter);
    GASSERT(curSize <= size);

    if (size < (curSize + ChecksumLen)) {
        return ErrorStatus::BufferOverflow;
    }

    auto checksum = calcChecksum(firstPosIter, curSize);
    Base::template writeData<ChecksumLen>(checksum, iter);
    return ErrorStatus::Success;
}

template <typename TTraits,
          typename TChecksumCalc,
          typename TNextLayer>
ErrorStatus ChecksumLayer<TTraits, TChecksumCalc, TNextLayer>::writeInternal(
    const MsgBase& msg,
    WriteIterator& iter,
    std::size_t size,
    const std::output_iterator_tag& tag) const
{
    static_cast<void>(tag);

    if (size < ChecksumLen) {
        return ErrorStatus::BufferOverflow;
    }

    auto status = Base::nextLayer().write(msg, iter, size - ChecksumLen);
    if ((status != ErrorStatus::Success) &&
        (status != ErrorStatus::UpdateRequired))
    {
        return status;
    }

    Base::template writeData<ChecksumLen>(0U, iter);
    return ErrorStatus::UpdateRequired;
}

}  // namespace protocol

}  // namespace comms

}  // namespace embxx
