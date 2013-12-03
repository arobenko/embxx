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


/// @file embxx/comms/protocol/MsgDataLayer.h
/// This file contains "Message Data Layer" protocol of "comms" module.

#pragma once

#include <cstddef>

#include "embxx/util/Assert.h"
#include "embxx/comms/traits.h"
#include "embxx/comms/ErrorStatus.h"

namespace embxx
{

namespace comms
{

namespace protocol
{

/// @ingroup comms
/// @brief Protocol layer forwards read/write requests to the message object.
/// @details This layer must be last in the defined protocol stack.
/// @tparam TMsgBase Base class for all the custom messages, smart pointer to
///         which is received by the read() member function.
/// @headerfile embxx/comms/protocol/MsgDataLayer.h
template <typename TMsgBase>
class MsgDataLayer
{
public:

    /// @brief Base class for all the messages
    typedef TMsgBase MsgBase;

    /// @brief Type of read iterator, defined in message base.
    typedef typename MsgBase::ReadIterator ReadIterator;

    /// @brief Type of write iterator, defined in message base.
    typedef typename MsgBase::WriteIterator WriteIterator;

    /// @brief MsgPtr is unknown type, will be redefined in one of other layers.
    typedef void MsgPtr;

    /// @brief Default constructor
    MsgDataLayer() = default;

    /// @brief Copy constructor is default
    MsgDataLayer(const MsgDataLayer&) = default;

    /// @brief Destructor is default
    ~MsgDataLayer() = default;

    /// @brief Deserialise message from the input data sequence.
    /// @details The function must receive smart pointer to already allocated
    ///          message object. It forwards the read() request to the latter.
    /// @param[in, out] msgPtr Reference to a smart pointer to allocated
    ///                 message object
    /// @param[in, out] iter Input iterator
    /// @param[in] size Size of the data in the buffer
    /// @param[out] missingSize If not nullptr and return value is
    ///             embxx::comms::ErrorStatus::NotEnoughData it will contain
    ///             minimal missing data length required for the successful
    ///             read attempt.
    /// @return Error status of the operation.
    /// @pre msgPtr must point to a valid message object.
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "size" times;
    /// @post The iterator will be advanced by the number of bytes was actually
    ///       read. In case of an error, distance between original position and
    ///       advanced will pinpoint the location of the error.
    /// @post missingSize output value is updated if and only if function
    ///       returns embxx::comms::ErrorStatus::NotEnoughData.
    /// @note Thread safety: Safe on distinct MsgIdLayer object and distinct
    ///       buffers, unsafe otherwise.
    /// @note Exception guarantee: Basic
    template <typename TMsgPtr>
    ErrorStatus read(
        TMsgPtr& msgPtr,
        ReadIterator& iter,
        std::size_t size,
        std::size_t* missingSize = nullptr);

    /// @brief Serialise message into the output data sequence.
    /// @details The function will forward the write() request to the provided
    ///          message object.
    /// @param[in] msg Reference to message object
    /// @param[in, out] iter Output iterator.
    /// @param[in] size Size of the buffer
    /// @return Status of the write operation.
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "size" times;
    /// @post The iterator will be advanced by the number of bytes was actually
    ///       read. In case of an error, distance between original position and
    ///       advanced will pinpoint the location of the error.
    /// @note Thread safety: Safe on distinct buffers, unsafe otherwise.
    /// @note Exception guarantee: Basic
    ErrorStatus write(
        const MsgBase& msg,
        WriteIterator& iter,
        std::size_t size) const;

    /// @brief Update the recently written output data sequence.
    /// @details Advances iterator "size" positions.
    template <typename TUpdateIter>
    ErrorStatus update(
        TUpdateIter& iter,
        std::size_t size) const;

    /// @brief Returns minimal protocol stack length required to serialise
    ///        empty message.
    /// @details This data layer is the last one protocol stack and doesn't
    ///          have any size overhead, 0 will be returned.
    constexpr std::size_t length() const;

    /// @brief Returns message serialisation length including protocol stack
    ///        overhead.
    ///        This data layer is the last on in protocol stack and doesn't
    ///        have any size overhead, result of msg.length() will be returned.
    std::size_t length(const MsgBase& msg) const;

};

// Implementation

template <typename TMsgBase>
template <typename TMsgPtr>
ErrorStatus MsgDataLayer<TMsgBase>::read(
    TMsgPtr& msgPtr,
    ReadIterator& iter,
    std::size_t size,
    std::size_t* missingSize)
{
    static_assert(std::is_base_of<MsgBase, typename std::decay<decltype(*msgPtr)>::type>::value,
        "TMsgBase must be a base class of decltype(*msgPtr)");

    GASSERT(msgPtr);
    auto result = msgPtr->read(iter, size);
    if ((result == ErrorStatus::NotEnoughData) &&
        (missingSize != nullptr)) {
        if (size < msgPtr->length()) {
            *missingSize = msgPtr->length() - size;
        }
        else {
            *missingSize = 1;
        }
    }
    return result;
}

template <typename TMsgBase>
ErrorStatus MsgDataLayer<TMsgBase>::write(
    const MsgBase& msg,
    WriteIterator& iter,
    std::size_t size) const
{
    return msg.write(iter, size);
}

template <typename TMsgBase>
template <typename TUpdateIter>
ErrorStatus MsgDataLayer<TMsgBase>::update(
    TUpdateIter& iter,
    std::size_t size) const
{
    std::advance(iter, size);
    return ErrorStatus::Success;
}

template <typename TMsgBase>
constexpr std::size_t MsgDataLayer<TMsgBase>::length() const
{
    return 0;
}

template <typename TMsgBase>
std::size_t MsgDataLayer<TMsgBase>::length(const MsgBase& msg) const
{
    return msg.length();
}

}  // namespace protocol

}  // namespace comms

}  // namespace embxx
