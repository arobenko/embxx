//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

#pragma once

#include <streambuf>

#include "embxx/util/Assert.h"
#include "embxx/comms/traits.h"
#include "embxx/comms/ErrorStatus.h"
#include "ProtocolLayer.h"

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

    /// @brief MsgPtr is unknown type, will be redefined in one of other layers.
    typedef void MsgPtr;

    /// @brief Default constructor
    MsgDataLayer() = default;

    /// @brief Deserialise message from the data in the input stream buffer.
    /// @details The function must receive smart pointer to already allocated
    ///          message object. It forwards the read() request to the latter.
    /// @param[in, out] msgPtr Reference to a smart pointer to allocated
    ///                 message object
    /// @param[in, out] buf Input stream buffer
    /// @param[in] size Size of the data in the buffer
    /// @return Error status of the operation.
    /// @pre msgPtr must point to a valid message object.
    /// @pre Value of provided "size" must be less than or equal to
    ///      available data in the buffer (size <= buf.in_avail());
    /// @post The internal (std::ios_base::in) pointer of the stream buffer
    ///       will be advanced by the number of bytes was actually read.
    ///       In case of an error, it will provide an information to the caller
    ///       about the place the error was recognised.
    /// @note Thread safety: Safe on distinct MsgIdLayer object and distinct
    ///       buffers, unsafe otherwise.
    /// @note Exception guarantee: Basic
    template <typename TMsgPtr>
    ErrorStatus read(TMsgPtr& msgPtr, std::streambuf& buf, std::size_t size);

    /// @brief Serialise message into the stream buffer.
    /// @details The function will forward the write() request to the provided
    ///          message object.
    /// @param[in] msg Reference to message object
    /// @param[in, out] buf Output stream buffer.
    /// @param[in] size size of the buffer
    /// @return Status of the write operation.
    /// @pre Value of provided "size" must be less than or equal to
    ///      available space in the buffer.
    /// @post The internal (std::ios_base::out) pointer of the stream buffer
    ///       will be advanced by the number of bytes was actually written.
    ///       In case of an error, it will provide an information to the caller
    ///       about the place the error was recognised.
    /// @note Thread safety: Safe on distinct stream buffers, unsafe otherwise.
    /// @note Exception guarantee: Basic
    ErrorStatus write(
        const MsgBase& msg,
        std::streambuf& buf,
        std::size_t size) const;
};

// Implementation


template <typename TMsgBase>
template <typename TMsgPtr>
ErrorStatus MsgDataLayer<TMsgBase>::read(
    TMsgPtr& msgPtr,
    std::streambuf& buf,
    std::size_t size)
{
    static_assert(std::is_base_of<MsgBase, typename std::decay<decltype(*msgPtr)>::type>::value,
        "TMsgBase must be a base class of decltype(*msgPtr)");

    GASSERT(size <= static_cast<decltype(size)>(buf.in_avail()));
    GASSERT(msgPtr);

    return msgPtr->read(buf, size);
}

template <typename TMsgBase>
ErrorStatus MsgDataLayer<TMsgBase>::write(
    const MsgBase& msg,
    std::streambuf& buf,
    std::size_t size) const
{
#ifndef NDEBUG
    auto firstPos = buf.pubseekoff(0, std::ios_base::cur, std::ios_base::out);
    auto lastPos = buf.pubseekoff(0, std::ios_base::end, std::ios_base::out);
    buf.pubseekpos(firstPos, std::ios_base::out);
    auto diff = static_cast<decltype(size)>(lastPos - firstPos);
    GASSERT(size <= diff);
#endif // #ifndef NDEBUG

    return msg.write(buf, size);
}

}  // namespace protocol

}  // namespace comms

}  // namespace embxx
