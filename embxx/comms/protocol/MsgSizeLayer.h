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

/// @file embxx/comms/protocol/MsgSizeLayer.h
/// This file contains "Message Size" protocol layer of the "comms" module.

#pragma once

#include <iterator>
#include <type_traits>
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
/// @brief Protocol layer that uses uses size field as a prefix to all the
///        subsequent data written by other (next) layers.
/// @details This layer is a mid level layer, expects other mid level layer or
///          MsgDataLayer to be its next one.
/// @tparam TTraits A traits class that must define:
///         @li Endianness type. Either embxx::comms::traits::endian::Big or
///             embxx::comms::traits::endian::Little
///         @li MsgSizeLen static integral constant of type std::size_t
///             specifying length of size field in bytes.
///         @li ExtraSizeValue static integral constant of type std::size_t
///             specifying extra value to be added to size field when
///             serialising
/// @tparam TNextLayer Next layer in the protocol stack
/// @headerfile embxx/comms/protocol/MsgSizeLayer.h
template <typename TTraits,
          typename TNextLayer>
class MsgSizeLayer : public ProtocolLayer<TTraits, TNextLayer>
{
    typedef ProtocolLayer<TTraits, TNextLayer> Base;

public:

    /// @brief Pointer to message object
    typedef typename Base::MsgPtr MsgPtr;

    /// @brief Base class to all custom messages
    typedef typename Base::MsgBase MsgBase;

    /// @brief Traits
    typedef typename Base::Traits Traits;

    /// @brief Length of the message size field. Originally defined in traits.
    static const std::size_t MsgSizeLen = Traits::MsgSizeLen;

    /// @brief Extra increment to the size value
    static const std::size_t ExtraSizeValue = Traits::ExtraSizeValue;

    /// Type of the "size" field
    typedef typename util::SizeToType<MsgSizeLen>::Type MsgSizeType;

    /// @brief Type of read iterator
    typedef typename Base::ReadIterator ReadIterator;

    /// @brief Type of write iterator
    typedef typename Base::WriteIterator WriteIterator;

    /// Constructor
    template <typename... TArgs>
    explicit MsgSizeLayer(TArgs&&... args);

    /// @brief Copy constructor is default
    MsgSizeLayer(const MsgSizeLayer&) = default;

    /// @brief Move constructor is default
    MsgSizeLayer(MsgSizeLayer&&) = default;

    /// @brief Copy assignment is default
    MsgSizeLayer& operator=(const MsgSizeLayer&) = default;

    /// @brief Move assignment is default.
    MsgSizeLayer& operator=(MsgSizeLayer&&) = default;

    /// @brief Destructor is default
    ~MsgSizeLayer() = default;


    /// @brief Deserialise message from the input data sequence.
    /// @details Reads size of the subsequent data from the input data sequence
    ///          and calls read() member function of the next layer with
    ///          the size specified in the size field.The function will also
    ///          compare the provided size of the data with size of the
    ///          message read from the buffer. If the latter is greater than
    ///          former, embxx::comms::ErrorStatus::NotEnoughData will be returned.
    ///          However, if buffer contains enough data, but the next layer
    ///          reports its not enough (returns embxx::comms::ErrorStatus::NotEnoughData),
    ///          embxx::comms::ErrorStatus::ProtocolError will be returned.
    /// @param[in, out] msgPtr Reference to smart pointer that already holds or
    ///                 will hold allocated message object
    /// @param[in, out] iter Input iterator.
    /// @param[in] size Size of the data in the sequence
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

    /// @brief Read the size information from the input data sequence.
    /// @details The read is executed from current input position of the buffer.
    ///          The internal pointer will NOT be returned to its original
    ///          position.
    /// @param[in, out] iter Input iterator
    /// @param[in] size Size of the data in the sequence
    /// @param[out] msgSize Value of the message size.
    /// @return Error status of the operation
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "size" times;
    /// @post The iterator will be advanced by the number of bytes was actually
    ///       read. In case of an error, distance between original position and
    ///       advanced will pinpoint the location of the error.
    /// @post The msgSize value is updated if and only if ErrorStatus::Success
    ///       is returned.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    static ErrorStatus readSize(
        ReadIterator& iter,
        std::size_t size,
        MsgSizeType& msgSize);

    /// @brief Serialise message into the output data sequence.
    /// @details The function will reserve space in the output data sequence
    ///          required to write size field, then forward the write() request
    ///          the the next layer in the protocol stack. After the latter
    ///          finishes its write, this protocol will evaluate the size of
    ///          the written data and update size field accordingly if such
    ///          update is possible (output iterator is random access one). If
    ///          update of the field is not possible (for example
    ///          std::back_insert_iterator was used), this function will return
    ///          embxx::comms::ErrorStatus::UpdateRequired. In this case
    ///          it is needed to call update() member function to finalise
    ///          the write operation.
    /// @param[in] msg Reference to message object
    /// @param[in, out] iter Output iterator.
    /// @param[in] size Available space in data sequence.
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
    /// @details Adds "MsgSizeLen" to the result of nextLayer().length().
    constexpr std::size_t length() const;

    /// @brief Returns message serialisation length including protocol stack
    ///        overhead.
    /// @details Adds "MsgSizeLen" to the result of nextLayer().length(msg).
    std::size_t length(const MsgBase& msg) const;

private:

    ErrorStatus write(
                const MsgBase& msg,
                WriteIterator& iter,
                std::size_t size,
                const std::random_access_iterator_tag& tag) const;

    ErrorStatus write(
                    const MsgBase& msg,
                    WriteIterator& iter,
                    std::size_t size,
                    const std::output_iterator_tag& tag) const;
};

// Implementation

template <typename TTraits, typename TNextLayer>
template <typename... TArgs>
MsgSizeLayer<TTraits, TNextLayer>::MsgSizeLayer(TArgs&&... args)
    : Base(std::forward<TArgs>(args)...)
{
}

template <typename TTraits, typename TNextLayer>
template <typename TMsgPtr>
ErrorStatus MsgSizeLayer<TTraits, TNextLayer>::read(
    TMsgPtr& msgPtr,
    ReadIterator& iter,
    std::size_t size,
    std::size_t* missingSize)
{
    static_assert(std::is_base_of<MsgBase, typename std::decay<decltype(*msgPtr)>::type>::value,
        "TMsgBase must be a base class of decltype(*msgPtr)");

    if (size < MsgSizeLen) {
        if (missingSize != nullptr) {
            *missingSize = length() - size;
        }
        return ErrorStatus::NotEnoughData;
    }

    MsgSizeType msgSize = 0;
    ErrorStatus status = readSize(iter, size, msgSize);
    static_cast<void>(status);
    GASSERT(status == ErrorStatus::Success);

    if ((0 < ExtraSizeValue) && (msgSize < ExtraSizeValue)) {
        return ErrorStatus::ProtocolError;
    }

    auto actualRemainingSize = (size - MsgSizeLen);
    auto requiredRemainingSize = (msgSize - ExtraSizeValue);
    if (actualRemainingSize < requiredRemainingSize) {
        if (missingSize != nullptr) {
            *missingSize = requiredRemainingSize - actualRemainingSize;
        }
        return ErrorStatus::NotEnoughData;
    }

    // not passing missingSize farther on purpose
    status = Base::nextLayer().read(msgPtr, iter, requiredRemainingSize);
    if (status == ErrorStatus::NotEnoughData) {
        return ErrorStatus::ProtocolError;
    }
    return status;
}

template <typename TTraits, typename TNextLayer>
ErrorStatus MsgSizeLayer<TTraits, TNextLayer>::readSize(
    ReadIterator& iter,
    std::size_t size,
    MsgSizeType& msgSize)
{
    if (size < MsgSizeLen) {
        return ErrorStatus::NotEnoughData;
    }

    msgSize = Base::template readData<MsgSizeType, MsgSizeLen>(iter);
    return ErrorStatus::Success;
}

template <typename TTraits, typename TNextLayer>
ErrorStatus MsgSizeLayer<TTraits, TNextLayer>::write(
    const MsgBase& msg,
    WriteIterator& iter,
    std::size_t size) const
{
    typedef typename std::iterator_traits<WriteIterator>::iterator_category IterType;
    return write(msg, iter, size, IterType());
}

template <typename TTraits, typename TNextLayer>
template <typename TUpdateIter>
ErrorStatus MsgSizeLayer<TTraits, TNextLayer>::update(
    TUpdateIter& iter,
    std::size_t size) const
{
    if (size < MsgSizeLen) {
        return ErrorStatus::BufferOverflow;
    }

    auto sizeToWrite = (size - MsgSizeLen) + ExtraSizeValue;
    Base::template writeData<MsgSizeLen>(sizeToWrite, iter);
    return Base::nextLayer().update(iter, size - MsgSizeLen);
}

template <typename TTraits, typename TNextLayer>
constexpr std::size_t MsgSizeLayer<TTraits, TNextLayer>::length() const
{
    return MsgSizeLen + Base::nextLayer().length();
}

template <typename TTraits, typename TNextLayer>
std::size_t MsgSizeLayer<TTraits, TNextLayer>::length(const MsgBase& msg) const
{
    return MsgSizeLen + Base::nextLayer().length(msg);
}

template <typename TTraits, typename TNextLayer>
ErrorStatus MsgSizeLayer<TTraits, TNextLayer>::write(
    const MsgBase& msg,
    WriteIterator& iter,
    std::size_t size,
    const std::random_access_iterator_tag& tag) const
{
    static_cast<void>(tag);
    WriteIterator firstIter(iter);
    if (size < MsgSizeLen) {
        return ErrorStatus::BufferOverflow;
    }

    Base::template writeData<MsgSizeLen>(0, iter);
    auto status = Base::nextLayer().write(msg, iter, size - MsgSizeLen);
    if (status == ErrorStatus::Success)
    {
        auto diff =
            (static_cast<std::size_t>(std::distance(firstIter, iter)) - MsgSizeLen) +
                                                                ExtraSizeValue;
        Base::template writeData<MsgSizeLen>(diff, firstIter);
    }

    return status;
}

template <typename TTraits, typename TNextLayer>
ErrorStatus MsgSizeLayer<TTraits, TNextLayer>::write(
    const MsgBase& msg,
    WriteIterator& iter,
    std::size_t size,
    const std::output_iterator_tag& tag) const
{
    static_cast<void>(tag);
    if (size < MsgSizeLen) {
        return ErrorStatus::BufferOverflow;
    }

    Base::template writeData<MsgSizeLen>(0U, iter);
    auto status = Base::nextLayer().write(msg, iter, size - MsgSizeLen);
    if (status != ErrorStatus::Success)
    {
        return status;
    }

    return ErrorStatus::UpdateRequired;
}

}  // namespace protocol

}  // namespace comms

}  // namespace embxx
