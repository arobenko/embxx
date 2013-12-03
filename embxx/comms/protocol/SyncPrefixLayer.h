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

/// @file embxx/comms/protocol/SyncPrefixLayer.h
/// This file contains defintion of "Sync Prefix" protocol layer of comms
/// module.


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
/// @brief Protocol layer that writes/expects "Sync" value prefix before
///        forwarding write/read requests to the next layers.
/// @details This layer is a mid level layer, expects other mid level layer or
///          MsgDataLayer to be its next one.
/// @tparam TTraits A traits class that must define:
///         @li Endianness type. Either embxx::comms::traits::endian::Big or
///             embxx::comms::traits::endian::Little
///         @li SyncPrefixLen static integral constant of type std::size_t
///             specifying length of the "sync prefix" field in bytes.
/// @tparam TNextLayer Next layer in the protocol stack
/// @headerfile embxx/comms/protocol/SyncPrefixLayer.h
template <typename TTraits, typename TNextLayer>
class SyncPrefixLayer : public ProtocolLayer<TTraits, TNextLayer>
{
    typedef ProtocolLayer<TTraits, TNextLayer> Base;

public:
    /// @brief Pointer to message object
    typedef typename Base::MsgPtr MsgPtr;

    /// @brief Base class to all custom messages
    typedef typename Base::MsgBase MsgBase;

    /// @brief Traits
    typedef typename Base::Traits Traits;

    /// @brief Length of the sync prefix field. Originally defined in traits.
    static const std::size_t SyncPrefixLen = Traits::SyncPrefixLen;

    /// Type of the "sync prefix" field
    typedef typename util::SizeToType<SyncPrefixLen>::Type SyncPrefixType;

    /// @brief Type of read iterator
    typedef typename Base::ReadIterator ReadIterator;

    /// @brief Type of write iterator
    typedef typename Base::WriteIterator WriteIterator;

    /// @brief Constructor
    /// @param sync Value of the expected sync prefix.
    /// @param args Parameters to be forwarded to next layer(s).
    template <typename... TArgs>
    explicit SyncPrefixLayer(SyncPrefixType sync, TArgs&&... args);

    /// @brief Copy constructor is default
    SyncPrefixLayer(const SyncPrefixLayer&) = default;

    /// @brief Move constructor is default
    SyncPrefixLayer(SyncPrefixLayer&&) = default;

    /// @brief Copy assignment is default
    SyncPrefixLayer& operator=(const SyncPrefixLayer&) = default;

    /// @brief Move assignment is default.
    SyncPrefixLayer& operator=(SyncPrefixLayer&&) = default;

    /// @brief Destructor is default
    ~SyncPrefixLayer() = default;

    /// @brief Get "sync prefix" value
    SyncPrefixType syncPrefix() const;


    /// @brief Deserialise message from the input data sequence.
    /// @details Reads SyncPrefixLen characters from the data sequence
    ///          and verifies they are as expected. If they match expected value
    ///          the read request is forwarded to the next layer, otherwise
    ///          embxx::comms::ErrorStatus::ProtocolError is returned.
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

    /// @brief Read the "sync" prefix from the input data sequence.
    /// @details The read is executed from current input position of the buffer.
    ///          The internal pointer will NOT be returned to its original
    ///          position.
    /// @param[in, out] iter Input iterator
    /// @param[in] size Size of the data in the sequence
    /// @param[out] sync Value of the sync prefix.
    /// @return Error status of the operation
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "size" times;
    /// @post The iterator will be advanced by the number of bytes was actually
    ///       read. In case of an error, distance between original position and
    ///       advanced will pinpoint the location of the error.
    /// @post The "sync" value is updated if and only if ErrorStatus::Success
    ///       is returned.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    static ErrorStatus readSync(
        ReadIterator& iter,
        std::size_t size,
        SyncPrefixType& sync);

    /// @brief Serialise message into the output data sequence.
    /// @details The function will write "sync prefix" to the data sequence and
    ///          then it will forward write() request to the next layer.
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
    /// @details Adds "SyncPrefixLen" to the result of nextLayer().length().
    constexpr std::size_t length() const;

    /// @brief Returns message serialisation length including protocol stack
    ///        overhead.
    /// @details Adds "SyncPrefixLen" to the result of nextLayer().length(msg).
    std::size_t length(const MsgBase& msg) const;

private:
    const SyncPrefixType sync_;

};

// Implementation
template <typename TTraits, typename TNextLayer>
template <typename... TArgs>
SyncPrefixLayer<TTraits, TNextLayer>::SyncPrefixLayer(
    SyncPrefixType sync,
    TArgs&&... args)
    : Base(std::forward<TArgs>(args)...),
      sync_(sync)
{
}

template <typename TTraits, typename TNextLayer>
typename SyncPrefixLayer<TTraits, TNextLayer>::SyncPrefixType
SyncPrefixLayer<TTraits, TNextLayer>::syncPrefix() const
{
    return sync_;
}

template <typename TTraits, typename TNextLayer>
template <typename TMsgPtr>
ErrorStatus SyncPrefixLayer<TTraits, TNextLayer>::read(
    TMsgPtr& msgPtr,
    ReadIterator& iter,
    std::size_t size,
    std::size_t* missingSize)
{
    static_assert(std::is_base_of<MsgBase, typename std::decay<decltype(*msgPtr)>::type>::value,
        "TMsgBase must be a base class of decltype(*msgPtr)");

    if (size < SyncPrefixLen) {
        if (missingSize != nullptr) {
            *missingSize = length() - size;
        }
        return ErrorStatus::NotEnoughData;
    }

    SyncPrefixType sync = 0;
    ErrorStatus status = readSync(iter, size, sync);
    static_cast<void>(status);
    GASSERT(status == ErrorStatus::Success);

    if (sync != sync_) {
        return ErrorStatus::ProtocolError;
    }

    return Base::nextLayer().read(msgPtr, iter, size - SyncPrefixLen, missingSize);
}

template <typename TTraits, typename TNextLayer>
ErrorStatus SyncPrefixLayer<TTraits, TNextLayer>::readSync(
    ReadIterator& iter,
    std::size_t size,
    SyncPrefixType& sync)
{

    if (size < SyncPrefixLen) {
        return ErrorStatus::NotEnoughData;
    }

    sync = Base::template readData<SyncPrefixType, SyncPrefixLen>(iter);
    return ErrorStatus::Success;
}

template <typename TTraits, typename TNextLayer>
ErrorStatus SyncPrefixLayer<TTraits, TNextLayer>::write(
    const MsgBase& msg,
    WriteIterator& iter,
    std::size_t size) const
{
    if (size < SyncPrefixLen) {
        return ErrorStatus::BufferOverflow;
    }

    Base::template writeData<SyncPrefixLen>(sync_, iter);
    return Base::nextLayer().write(msg, iter, size - SyncPrefixLen);
}

template <typename TTraits, typename TNextLayer>
template <typename TUpdateIter>
ErrorStatus SyncPrefixLayer<TTraits, TNextLayer>::update(
    TUpdateIter& iter,
    std::size_t size) const
{
    if (size < SyncPrefixLen) {
        return ErrorStatus::BufferOverflow;
    }

    std::advance(iter, SyncPrefixLen);
    return Base::nextLayer().update(iter, size - SyncPrefixLen);
}

template <typename TTraits, typename TNextLayer>
constexpr
std::size_t SyncPrefixLayer<TTraits, TNextLayer>::length() const
{
    return SyncPrefixLen + Base::nextLayer().length();
}

template <typename TTraits, typename TNextLayer>
std::size_t SyncPrefixLayer<TTraits, TNextLayer>::length(
    const MsgBase& msg) const
{
    return SyncPrefixLen + Base::nextLayer().length(msg);
}


}  // namespace protocol

}  // namespace comms

}  // namespace embxx


