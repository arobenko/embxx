//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

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

    /// Constructor
    MsgSizeLayer() = default;

    /// @brief Deserialise message from the data in the input stream buffer.
    /// @details Reads size of the subsequent data in the stream buffer
    ///          and calls read() member function of the next layer with
    ///          the size specified in the size field.The function will also
    ///          compare the provided size of the buffer with size of the
    ///          message read from the buffer. If the latter is greater than
    ///          former, embxx::comms::ErrorStatus::NotEnoughData will be returned.
    ///          However, if buffer contains enough data, but the next layer
    ///          reports its not enough (returns embxx::comms::ErrorStatus::NotEnoughData),
    ///          embxx::comms::ErrorStatus::ProtocolError will be returned.
    /// @param[in, out] msgPtr Reference to smart pointer that already holds or
    ///                 will hold allocated message object
    /// @param[in, out] buf Input stream buffer
    /// @param[in] size Size of the data in the buffer
    /// @return Error status of the operation.
    /// @pre Value of provided "size" must be less than or equal to
    ///      available data in the buffer.
    /// @post The internal (std::ios_base::in) pointer of the stream buffer
    ///       will be advanced by the number of bytes actually read.
    ///       In case of an error, it will provide an information to the caller
    ///       about the place the error was recognised.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    template <typename TMsgPtr>
    ErrorStatus read(TMsgPtr& msgPtr, std::streambuf& buf, std::size_t size);

    /// @brief Read the size information from the stream buffer.
    /// @details The read is executed from current input position of the buffer.
    ///          The internal pointer will NOT be returned to its original
    ///          position.
    /// @param[in, out] buf Input stream buffer
    /// @param[in] size Size of the data in the buffer
    /// @param[out] msgSize Value of the message size.
    /// @return Error status of the operation
    /// @pre Value of provided "size" must be less than or equal to
    ///      available data in the buffer.
    /// @post The internal (std::ios_base::in) pointer of the stream buffer
    ///       will be advanced by the number of bytes actually read.
    ///       In case of an error, it will provide an information to the caller
    ///       about the place the error was recognised.
    /// @post The msgSize value is updated if and only if ErrorStatus::Success
    ///       is returned.
    ///       pointer points to a valid object.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    static ErrorStatus readSize(
        std::streambuf& buf,
        std::size_t size,
        MsgSizeType& msgSize);

    /// @brief Serialise message into the stream buffer.
    /// @details The function will reserve space in the output stream buffer
    ///          required to write size field, then forward the write() request
    ///          the the next layer in the protocol stack. After the latter
    ///          finishes its write, this protocol will evaluate the size of
    ///          the written data and update size field accordingly.
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
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Basic
    ErrorStatus write(
            const MsgBase& msg,
            std::streambuf& buf,
            std::size_t size) const;

};

// Implementation

template <typename TTraits, typename TNextLayer>
template <typename TMsgPtr>
ErrorStatus MsgSizeLayer<TTraits, TNextLayer>::read(
    TMsgPtr& msgPtr,
    std::streambuf& buf,
    std::size_t size)
{
    static_assert(std::is_base_of<MsgBase, typename std::decay<decltype(*msgPtr)>::type>::value,
        "TMsgBase must be a base class of decltype(*msgPtr)");

    GASSERT(size <= static_cast<decltype(size)>(buf.in_avail()));

    if (size < (MsgSizeLen + ExtraSizeValue)) {
        return ErrorStatus::NotEnoughData;
    }

    MsgSizeType msgSize = 0;
    ErrorStatus status = readSize(buf, size, msgSize);
    if (status != ErrorStatus::Success) {
        return status;
    }

    if ((0 < ExtraSizeValue) && (msgSize < ExtraSizeValue)) {
        return ErrorStatus::ProtocolError;
    }

    if ((size - MsgSizeLen) < (msgSize - ExtraSizeValue)) {
        return ErrorStatus::NotEnoughData;
    }

    status = Base::nextLayer().read(msgPtr, buf, msgSize - ExtraSizeValue);
    if (status == ErrorStatus::NotEnoughData) {
        return ErrorStatus::ProtocolError;
    }
    return status;
}

template <typename TTraits, typename TNextLayer>
ErrorStatus MsgSizeLayer<TTraits, TNextLayer>::readSize(
    std::streambuf& buf,
    std::size_t size,
    MsgSizeType& msgSize)
{
    GASSERT(size <= static_cast<decltype(size)>(buf.in_avail()));
    if (size < MsgSizeLen) {
        return ErrorStatus::NotEnoughData;
    }

    msgSize = Base::template getData<MsgSizeType, MsgSizeLen>(buf);
    return ErrorStatus::Success;
}

template <typename TTraits, typename TNextLayer>
ErrorStatus MsgSizeLayer<TTraits, TNextLayer>::write(
    const MsgBase& msg,
    std::streambuf& buf,
    std::size_t size) const
{
    auto firstPos = buf.pubseekoff(0, std::ios_base::cur, std::ios_base::out);

#ifndef NDEBUG
    auto lastPos = buf.pubseekoff(0, std::ios_base::end, std::ios_base::out);
    buf.pubseekpos(firstPos, std::ios_base::out);
    auto diff = static_cast<decltype(size)>(lastPos - firstPos);
    GASSERT(size <= diff);
#endif // #ifndef NDEBUG

    if (size < MsgSizeLen) {
        return ErrorStatus::BufferOverflow;
    }

    Base::template putData<MsgSizeLen>(0, buf);
    auto status = Base::nextLayer().write(msg, buf, size - MsgSizeLen);
    if (status == ErrorStatus::Success)
    {
        auto curPos = buf.pubseekoff(0, std::ios_base::cur, std::ios_base::out);
        auto diff =
            (static_cast<std::size_t>(curPos - firstPos) - MsgSizeLen) +
                                                                ExtraSizeValue;
        buf.pubseekpos(firstPos, std::ios_base::out);
        Base::template putData<MsgSizeLen>(diff, buf);
        buf.pubseekpos(curPos, std::ios_base::out);
    }

    return status;
}

}  // namespace protocol

}  // namespace comms

}  // namespace embxx
