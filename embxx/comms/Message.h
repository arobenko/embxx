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

/// @file embxx/comms/Message.h
/// Contains definition of Message object interface and various base classes
/// for custom messages.

#pragma once

#include <cstdint>
#include <algorithm>
#include <type_traits>

#include "embxx/util/Assert.h"
#include "embxx/util/Tuple.h"
#include "embxx/io/access.h"

#include "traits.h"
#include "ErrorStatus.h"

namespace embxx
{

namespace comms
{

/// @addtogroup comms
/// @{

/// @brief Abstract base class for all the messages.
/// @details This class provides basic API to allow retrieval of message ID,
///          reading and writing of the message contents, as well as dispatching
///          message to its handler.
/// @tparam THandler Handler class that must provide
///         "handleMessage(<message_type>& msg)" member functions to handle
///         all types of defined messages.
/// @tparam TTraits Various behavioural traits relevant for the message. Must
///         define:
///         @li Type Endianness. Must be either embxx::comms::traits::endian::Big
///             or embxx::comms::traits::endian::Little.
///         @li Type ReadIterator. Can be any type of input iterator. It will
///             be used to read message contents from serialised data sequence.
///         @li Type WriteIterator. Can be any type of output iterator. It will
///             be used to serialise message contents to provided data sequence.
/// @headerfile embxx/comms/Message.h
template <typename THandler, typename TTraits>
class Message
{
public:

    /// @brief Handler class.
    /// @details Must provide "handleMessage(...)" functions.
    typedef THandler Handler;

    /// @brief Traits class type.
    typedef TTraits Traits;

    /// @brief Type used for message ID
    typedef typename traits::MsgIdType MsgIdType;

    /// Actual Endianness defined in provided Traits class
    typedef typename Traits::Endianness Endianness;

    /// @brief Type of read iterator
    typedef typename Traits::ReadIterator ReadIterator;

    /// @brief Type of write iterator
    typedef typename Traits::WriteIterator WriteIterator;

    /// @brief Destructor
    virtual ~Message();

    /// @brief Retrieve ID of the message
    /// @note Thread safety: Safe
    /// @note Exception guarantee: Strong
    MsgIdType getId() const;

    /// @brief Read body of the message from stream buffer
    /// @details Calls to pure virtual function readImpl() which must
    ///          be implemented in one of the derived classes. Prior to call
    ///          to readImpl(), call to length() is performed to
    ///          verify that buffer contains enough data to create a message.
    ///          If the amount of data in the buffer is not enough
    ///          ErrorStatus::NotEnoughData will be returned.
    /// @param[in, out] iter Input iterator.
    /// @param[in] size Size of the data in iterated data structure.
    /// @return Status of the read operation.
    /// @pre Input iterator must be valid and can be successfully dereferenced
    ///      and incremented at least size times.
    /// @post The input iterator is advanced.
    /// @post There is no guarantee about state of the message object after
    ///       the read operation is not successful. It may be only partially
    ///       updated.
    /// @note Thread safety: Depends on thread safety of the implementation of
    ///       readImpl() provided by the derived class(es).
    /// @note Exception guarantee: Basic
    ErrorStatus read(ReadIterator& iter, std::size_t size);

    /// @brief Write body of the message to the stream buffer
    /// @details This function checks whether required buffer size returned
    ///          by length() is less or equal to the "size" parameter
    ///          value. In case there is not enough space this function will
    ///          return ErrorStatus::BufferOverflow. It there is enough space
    ///          to successfully write the message, pure virtual function
    ///          writeImpl() will be called. It must
    ///          be implemented in one of the derived classes.
    /// @param[in, out] iter Output iterator.
    /// @param[in] size Size of the buffer, message data must fit it.
    /// @return Status of the write operation.
    /// @pre Input iterator must be valid and can be successfully dereferenced
    ///      and incremented at least size times.
    /// @post The output iterator is advanced.
    /// @note Thread safety: Depends on thread safety of the implementation of
    ///       writeImpl() provided by the derived class(es).
    /// @note Exception guarantee: Basic
    ErrorStatus write(WriteIterator& iter, std::size_t size) const;

    /// @brief Get size required to serialise a message.
    /// @details This function will call lengthImpl() pure virtual
    ///          function. It is a responsibility of the actual message
    ///          to implement it to provide the information.
    /// @return Number of bytes required to serialise a message.
    /// @post Returned value is less than std::numeric_limits<std::size_t>::max();
    /// @note Thread safety: Depends on thread safety of the implementation of
    ///       lengthImpl() provided by the derived class(es).
    /// @note Exception guarantee: No throw.
    std::size_t length() const;

    /// @brief Dispatch message to its handler
    /// @details The message will be dispatched to the handler using
    ///          appropriate "handleMessage(...) member function of
    ///          the latter.
    /// @param[in] handler Reference to the handler object
    /// @pre The handler must define member functions with one of the following
    ///      signatures for all the messages it needs to handle.
    ///      @li void handleMessage(<message_type>& msg);
    ///      @li void handleMessage(const <message_type>& msg);
    /// @note Thread safety: Depends on thread safety of handler's code.
    /// @note Exception guarantee: Same as exception guarantee of the handler's
    ///       code
    void dispatch(Handler& handler);

protected:
    /// @brief Pure virtual function to be called to update contents of the
    ///        message based on the data in the stream buffer.
    /// @details Must be implemented in the derived class
    /// @param[in, out] iter Input iterator.
    /// @param[in] size Size of the data in the iterated sequence.
    /// @return Status of the read operation.
    /// @note Must comply with all the preconditions, postconditions,
    ///       Thread safety and Exception guarantee specified in read().
    virtual ErrorStatus readImpl(ReadIterator& iter, std::size_t size) = 0;

    /// @brief Pure virtual function to be called to write contents of the
    ///        message to the stream buffer.
    /// @details Must be implemented in the derived class
    /// @param[in, out] iter Output stream buffer.
    /// @param[in] size Maximal size the message data must fit it.
    /// @return Status of the write operation.
    /// @pre lengthImpl() <= size. This check is performed in write()
    ///      prior to call to this function.
    /// @note Must comply with all the preconditions, postconditions,
    ///       Thread safety and Exception guarantee specified in write().
    virtual ErrorStatus writeImpl(WriteIterator& iter, std::size_t size) const = 0;

    /// @brief Pure virtual function to be called to retrieve
    ///        number of bytes required to serialise current message.
    /// @details Must be implemented in the derived class.
    /// @return Number of bytes required to serialise a message.
    /// @note Must comply with all the preconditions, postconditions,
    ///       Thread safety and Exception guarantee specified in length().
    virtual std::size_t lengthImpl() const = 0;

    /// @brief Write data into the output sequence.
    /// @details Use this function to write data to the stream buffer.
    ///          The endianness of the data will be as specified in the TTraits
    ///          template parameter of the class.
    /// @tparam T Type of the value to write. Must be integral.
    /// @tparam Type of output iterator
    /// @param[in] value Integral type value to be written.
    /// @param[in, out] iter Output iterator.
    /// @pre The iterator must be valid and can be successfully dereferenced
    ///      and incremented at least sizeof(T) times.
    /// @post The iterator is advanced.
    /// @note Thread safety: Safe for distinct buffers, unsafe otherwise.
    template <typename T, typename TIter>
    static void writeData(T value, TIter& iter);

    /// @brief Write partial data into the output sequence.
    /// @details Use this function to write partial data to the stream buffer.
    ///          The endianness of the data will be as specified in the TTraits
    ///          template parameter of the class.
    /// @tparam TSize Length of the value in bytes known in compile time.
    /// @tparam T Type of the value to write. Must be integral.
    /// @tparam TIter Type of output iterator
    /// @param[in] value Integral type value to be written.
    /// @param[in, out] iter Output iterator.
    /// @pre TSize <= sizeof(T)
    /// @pre The iterator must be valid and can be successfully dereferenced
    ///      and incremented at least TSize times.
    /// @post The iterator is advanced.
    /// @note Thread safety: Safe for distinct buffers, unsafe otherwise.
    template <std::size_t TSize, typename T, typename TIter>
    static void writeData(T value, TIter& iter);

    /// @brief Read data from input sequence.
    /// @details Use this function to read data from the stream buffer.
    /// The endianness of the data will be as specified in the TTraits
    /// template parameter of the class.
    /// @tparam T Return type
    /// @tparam TIter Type of input iterator
    /// @param[in, out] iter Input iterator.
    /// @return The integral type value.
    /// @pre TSize <= sizeof(T)
    /// @pre The iterator must be valid and can be successfully dereferenced
    ///      and incremented at least sizeof(T) times.
    /// @post The iterator is advanced.
    /// @note Thread safety: Safe for distinct stream buffers, unsafe otherwise.
    template <typename T, typename TIter>
    static T readData(TIter& iter);

    /// @brief Read partial data from input sequence.
    /// @details Use this function to read data from the stream buffer.
    /// The endianness of the data will be as specified in the TTraits
    /// template parameter of the class.
    /// @tparam T Return type
    /// @tparam TSize number of bytes to read
    /// @tparam TIter Type of input iterator
    /// @param[in, out] iter Input iterator.
    /// @return The integral type value.
    /// @pre TSize <= sizeof(T)
    /// @pre The iterator must be valid and can be successfully dereferenced
    ///      and incremented at least TSize times.
    /// @post The internal pointer of the stream buffer is advanced.
    /// @note Thread safety: Safe for distinct stream buffers, unsafe otherwise.
    template <typename T, std::size_t TSize, typename TIter>
    static T readData(TIter& iter);

private:

    /// @brief Implemented in MessageBase
    virtual MsgIdType getIdImpl() const = 0;

    /// @brief Implemented in MessageBase
    virtual void dispatchImpl(Handler& handler) = 0;
};

/// @brief Base class for all the custom messages.
/// @details Every project specific custom message definition class must be
///          derived from this class. It provides the common facility to
///          retrieve the ID of the message and dispatch message to the
///          appropriate function in the handler.
/// @tparam TId ID of the message (numeric value).
/// @tparam TBase Common base class for all the messages that exposes the
///         required interface. TBase must be derived directly from the
///         abstract Message class and may add extra project specific
///         API functions.
/// @tparam TActual Actual message being defined that is derived from this
///         class
/// @extends Message
/// @headerfile embxx/comms/Message.h
template <traits::MsgIdType TId,
          typename TBase,
          typename TActual>
class MessageBase : public TBase
{
    typedef TBase Base;
    typedef TActual Actual;

public:

    /// @brief Type used for message ID
    typedef typename Base::MsgIdType MsgIdType;

    /// @brief Type of the message handler
    typedef typename Base::Handler Handler;

    /// @brief Compile time known ID of the message
    static const MsgIdType MsgId = TId;

    /// @brief Destructor
    virtual ~MessageBase();

private:

    /// Returns MsgId
    virtual MsgIdType getIdImpl() const override final;

    virtual void dispatchImpl(Handler& handler) override final;
};

/// @brief Base class to support definition of messages with empty body.
/// @details Implements default empty behaviour for readImpl() and writeImpl()
///          member functions. Inherits from MessageBase, all template
///          parameters are forwarded to the base class.
/// @headerfile embxx/comms/Message.h
template <traits::MsgIdType TId,
          typename TBase,
          typename TActual>
class EmptyBodyMessage : public MessageBase<TId, TBase, TActual>
{
    typedef MessageBase<TId, TBase, TActual> Base;

public:

    // Destructor
    virtual ~EmptyBodyMessage();

protected:

    /// @brief Read iterator type
    typedef typename Base::ReadIterator ReadIterator;

    /// @brief Write iterator type
    typedef typename Base::WriteIterator WriteIterator;

    /// @brief Implements read body behaviour.
    /// @details Does nothing
    /// @return ErrorStatus::Success.
    virtual ErrorStatus readImpl(
        ReadIterator& iter,
        std::size_t size) override final;

    /// @brief Implements read body behaviour.
    /// @details Does nothing
    /// @return ErrorStatus::Success.
    virtual ErrorStatus writeImpl(
        WriteIterator& iter,
        std::size_t size) const override final;

    /// @brief Implements serialisation size retrieval.
    /// @return 0
    virtual std::size_t lengthImpl() const override final;
};

/// @brief Meta base class for all the custom messages.
/// @details The only difference between this class and MessageBase is that
///          this class receives all the fields wrapped in std::tuple and
///          manages all read/write/getDataLength operations for these
///          fields.
/// @tparam TId ID of the message (numeric value).
/// @tparam TBase Common base class for all the messages that exposes the
///         required interface. TBase must be derived directly from the
///         abstract Message class and may add extra project specific
///         API functions.
/// @tparam TActual Actual message being defined that is derived from this
///         class
/// @tparam TFields All field classes wrapped in std::tuple. Every field
///         class must provide the following member functions:
///         @li @code std::size_t length() const; @endcode Provides an
///             information about length of the field when serialised. May also
///             be static if doesn't depend on the field value.
///         @li @code template <typename TIter> ErrorStatus read(TIter& iter, std::size_t size); @endcode
///             Reads the serialised value from the input data sequence and
///             updates its value.
///         @li @code template <typename TIter> ErrorStatus write(TIter& iter, std::size_t size) const; @endcode
///             Serialises value into the output data sequence.
/// @headerfile embxx/comms/Message.h
template <traits::MsgIdType TId,
          typename TBase,
          typename TActual,
          typename TFields>
class MetaMessageBase : public MessageBase<TId, TBase, TActual>
{
    typedef MessageBase<TId, TBase, TActual> Base;

    static_assert(util::IsTuple<TFields>::Value, "TFields must be std::tuple");

public:

    /// @brief Fields tuple
    typedef TFields Fields;

    /// @brief Default constructor
    MetaMessageBase() = default;

    explicit MetaMessageBase(const Fields& fields);

    explicit MetaMessageBase(Fields&& fields);

    /// @brief Destructor
    virtual ~MetaMessageBase();

    /// @brief Access all the fields
    Fields& getFields();

    /// @brief Const version of getFields
    const Fields& getFields() const;

protected:

    /// @brief Read iterator type is inherited from base class
    typedef typename Base::ReadIterator ReadIterator;

    /// @brief Write iterator type is inherited from base class
    typedef typename Base::WriteIterator WriteIterator;

    /// @brief Implements read body behaviour.
    /// @details Calls read() member function of every element in TFields.
    /// @return ErrorStatus::Success if all read operations are successful.
    virtual ErrorStatus readImpl(
        ReadIterator& iter,
        std::size_t size) override;

    /// @brief Implements write body behaviour.
    /// @details Calls write() member function of every element in TField
    /// @return ErrorStatus::Success if all write operations are successful.
    virtual ErrorStatus writeImpl(
        WriteIterator& iter,
        std::size_t size) const override;

    /// @brief Implements serialisation size retrieval.
    /// @details Calls length() member functio nof every element it TField
    ///          and sums the result
    /// @return Number of bytes required to serialise all fields in TField
    virtual std::size_t lengthImpl() const override;

private:

    /// @cond DOCUMENT_FIELD_READER_WRITER_SIZE_GETTER
    class FieldReader
    {
    public:
        FieldReader(ReadIterator& iter, ErrorStatus& status, std::size_t& size)
            : iter_(iter),
              status_(status),
              size_(size)
        {

        }

        template <typename TField>
        void operator()(TField& field) {
            if (status_ == ErrorStatus::Success) {
                status_ = field.read(iter_, size_);
                if (status_ == ErrorStatus::Success) {
                    GASSERT(field.length() <= size_);
                    size_ -= field.length();
                }
            }
        }
    private:
        ReadIterator& iter_;
        ErrorStatus status_;
        std::size_t& size_;
    };

    class FieldWriter
    {
    public:
        FieldWriter(WriteIterator& iter, ErrorStatus& status, std::size_t& size)
            : iter_(iter),
              status_(status),
              size_(size)
        {
        }

        template <typename TField>
        void operator()(TField& field) {
            if (status_ == ErrorStatus::Success) {
                status_ = field.write(iter_, size_);
                if (status_ == ErrorStatus::Success) {
                    GASSERT(field.length() <= size_);
                    size_ -= field.length();
                }
            }
        }

    private:
        WriteIterator& iter_;
        ErrorStatus status_;
        std::size_t& size_;
    };


    struct FieldLengthRetriever
    {
        template <typename TField>
        std::size_t operator()(std::size_t size, const TField& field)
        {
            return size + field.length();
        }
    };
    /// @endcond

    Fields fields_;
};


/// @}

// Implementation
template <typename THandler, typename TTraits>
Message<THandler, TTraits>::~Message()
{
}

template <typename THandler, typename TTraits>
typename Message<THandler, TTraits>::MsgIdType
Message<THandler, TTraits>::getId() const
{
    // redirecting request to derived class
    return this->getIdImpl();
}

template <typename THandler, typename TTraits>
ErrorStatus Message<THandler, TTraits>::read(
    ReadIterator& iter,
    std::size_t size)
{
    auto minSize = length();
    if (size < minSize) {
        return ErrorStatus::NotEnoughData;
    }

    return this->readImpl(iter, size);
}

template <typename THandler, typename TTraits>
ErrorStatus Message<THandler, TTraits>::write(
    WriteIterator& iter,
    std::size_t size) const
{
    if (size < length()) {
        return ErrorStatus::BufferOverflow;
    }

    return this->writeImpl(iter, size);
}

template <typename THandler, typename TTraits>
std::size_t Message<THandler, TTraits>::length() const
{
    auto value = this->lengthImpl();
    GASSERT(value < std::numeric_limits<decltype(value)>::max());
    return value;
}


template <typename THandler, typename TTraits>
void Message<THandler, TTraits>::dispatch(Handler& handler)
{
    this->dispatchImpl(handler);
}

template <typename THandler, typename TTraits>
template <typename T, typename TIter>
void Message<THandler, TTraits>::writeData(
    T value,
    TIter& iter)
{
    io::writeData<T>(value, iter, Endianness());
}

template <typename THandler, typename TTraits>
template <std::size_t TSize, typename T, typename TIter>
void Message<THandler, TTraits>::writeData(
    T value,
    TIter& iter)
{
    static_assert(TSize <= sizeof(T),
                                "Cannot put more bytes than type contains");
    return io::writeData<TSize, T>(value, iter, Endianness());
}

template <typename THandler, typename TTraits>
template <typename T, typename TIter>
T Message<THandler, TTraits>::readData(TIter& iter)
{
    return readData<T, sizeof(T)>(iter);
}

template <typename THandler, typename TTraits>
template <typename T, std::size_t TSize, typename TIter>
T Message<THandler, TTraits>::readData(TIter& iter)
{
    static_assert(TSize <= sizeof(T),
        "Cannot get more bytes than type contains");
    return io::readData<T, TSize>(iter, Endianness());
}

template <typename traits::MsgIdType TId,
          typename TBase,
          typename TActual>
MessageBase<TId, TBase, TActual>::~MessageBase()
{
}

template <typename traits::MsgIdType TId,
          typename TBase,
          typename TActual>
typename MessageBase<TId, TBase, TActual>::MsgIdType
MessageBase<TId, TBase, TActual>::getIdImpl() const
{
    return MsgId;
}

template <typename traits::MsgIdType TId,
          typename TBase,
          typename TActual>
void MessageBase<TId, TBase, TActual>::dispatchImpl(Handler& handler)
{
    Actual& actual = static_cast<Actual&>(*this);
    handler.handleMessage(actual);
}

template <typename traits::MsgIdType TId,
          typename TBase,
          typename TActual>
EmptyBodyMessage<TId, TBase, TActual>::~EmptyBodyMessage()
{
}

template <typename traits::MsgIdType TId,
          typename TBase,
          typename TActual>
ErrorStatus EmptyBodyMessage<TId, TBase, TActual>::readImpl(
    ReadIterator& iter,
    std::size_t size)
{
    static_cast<void>(iter);
    static_cast<void>(size);
    return ErrorStatus::Success;
}

template <typename traits::MsgIdType TId,
          typename TBase,
          typename TActual>
ErrorStatus EmptyBodyMessage<TId, TBase, TActual>::writeImpl(
    WriteIterator& iter,
    std::size_t size) const
{
    static_cast<void>(iter);
    static_cast<void>(size);
    return ErrorStatus::Success;
}

template <typename traits::MsgIdType TId,
          typename TBase,
          typename TActual>
std::size_t EmptyBodyMessage<TId, TBase, TActual>::lengthImpl() const
{
    return 0;
}

template <traits::MsgIdType TId,
          typename TBase,
          typename TActual,
          typename TFields>
MetaMessageBase<TId, TBase, TActual, TFields>::MetaMessageBase(
    const Fields& fields)
    : fields_(fields)
{
}

template <traits::MsgIdType TId,
          typename TBase,
          typename TActual,
          typename TFields>
MetaMessageBase<TId, TBase, TActual, TFields>::MetaMessageBase(
    Fields&& fields)
    : fields_(std::move(fields))
{
}

template <traits::MsgIdType TId,
          typename TBase,
          typename TActual,
          typename TFields>
MetaMessageBase<TId, TBase, TActual, TFields>::~MetaMessageBase()
{
}

template <traits::MsgIdType TId,
          typename TBase,
          typename TActual,
          typename TFields>
typename MetaMessageBase<TId, TBase, TActual, TFields>::Fields&
MetaMessageBase<TId, TBase, TActual, TFields>::getFields()
{
    return fields_;
}

template <traits::MsgIdType TId,
          typename TBase,
          typename TActual,
          typename TFields>
const typename MetaMessageBase<TId, TBase, TActual, TFields>::Fields&
MetaMessageBase<TId, TBase, TActual, TFields>::getFields() const
{
    return fields_;
}

template <traits::MsgIdType TId,
          typename TBase,
          typename TActual,
          typename TFields>
ErrorStatus MetaMessageBase<TId, TBase, TActual, TFields>::readImpl(
    ReadIterator& iter,
    std::size_t size)
{
    ErrorStatus status = ErrorStatus::Success;
    std::size_t remainingSize = size;
    util::tupleForEach(fields_, FieldReader(iter, status, remainingSize));
    return status;
}

template <traits::MsgIdType TId,
          typename TBase,
          typename TActual,
          typename TFields>
ErrorStatus MetaMessageBase<TId, TBase, TActual, TFields>::writeImpl(
    WriteIterator& iter,
    std::size_t size) const
{
    ErrorStatus status = ErrorStatus::Success;
    std::size_t remainingSize = size;
    util::tupleForEach(fields_, FieldWriter(iter, status, remainingSize));
    return status;
}

template <traits::MsgIdType TId,
          typename TBase,
          typename TActual,
          typename TFields>
std::size_t MetaMessageBase<TId, TBase, TActual, TFields>::lengthImpl() const
{
    return util::tupleAccumulate(fields_, 0U, FieldLengthRetriever());
}


}  // namespace comms

}  // namespace embxx
