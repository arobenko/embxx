//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file comms/Message.h
/// Contains definition of Message object interface and various base classes
/// for custom messages.

#pragma once

#include <streambuf>
#include <cstdint>
#include <algorithm>
#include <type_traits>

#include "embxx/util/Assert.h"
#include "embxx/util/Tuple.h"
#include "embxx/io/iosimple.h"

#include "traits.h"
#include "ErrorStatus.h"

namespace embxx
{

namespace comms
{

/// @addtogroup comms
/// @{

/// @brief Abstract base class for all the messages.
/// @details This class is used to avoid multiple instantiations of the same
///          template arguments independent code.
/// @headerfile embxx/comms/Message.h
template <typename T = void>
class CommonMessageBase
{
public:

    /// @brief Type used for message ID
    typedef traits::MsgIdType MsgIdType;

    /// @brief Destructor
    virtual ~CommonMessageBase();

    /// @brief Retrieve ID of the message
    /// @note Thread safety: Safe
    /// @note Exception guarantee: Strong
    MsgIdType getId() const;

    /// @brief Read body of the message from stream buffer
    /// @details Calls to pure virtual function readImpl() which must
    ///          be implemented in one of the derived classes. Prior to call
    ///          to readImpl(), call to getDataSize() is performed to
    ///          verify that buffer contains enough data to create a message.
    ///          If the amount of data in the buffer is not enough
    ///          ErrorStatus::NotEnoughData will be returned.
    /// @param[in, out] buf Input stream buffer.
    /// @param[in] size Size of the data in the stream buffer.
    /// @return Status of the read operation.
    /// @pre Value of provided "size" must be less than or equal to
    ///      available data in the buffer
    /// @post The internal (std::ios_base::in) pointer of the stream buffer
    ///       will be advanced by the number of bytes was actually read.
    ///       In case of an error, it will provide an information to the caller
    ///       about the place the error was recognised.
    /// @post There is no guarantee about state of the message object after
    ///       the read operation is not successful. It may be only partially
    ///       updated.
    /// @note Thread safety: Depends on thread safety of the implementation of
    ///       readImpl() provided by the derived class(es).
    /// @note Exception guarantee: Basic
    ErrorStatus read(std::streambuf& buf, std::size_t size);

    /// @brief Write body of the message to the stream buffer
    /// @details This function checks whether required buffer size returned
    ///          by getDataSize() is less or equal to the "size" parameter
    ///          value. In case there is not enough space this function will
    ///          return ErrorStatus::BufferOverflow. It there is enough space
    ///          to successfully write the message, pure virtual function
    ///          writeImpl() will be called. It must
    ///          be implemented in one of the derived classes.
    /// @param[in, out] buf Output stream buffer.
    /// @param[in] size Size of the buffer, message data must fit it.
    /// @return Status of the write operation.
    /// @pre Value of provided "size" must be less than or equal to
    ///      available space in the buffer.
    /// @post The internal (std::ios_base::out) pointer of the stream buffer
    ///       will be advanced by the number of bytes was actually written.
    ///       In case of an error, it will provide an information to the caller
    ///       about the place the error was recognised.
    /// @note Thread safety: Depends on thread safety of the implementation of
    ///       writeImpl() provided by the derived class(es).
    /// @note Exception guarantee: Basic
    ErrorStatus write(std::streambuf& buf, std::size_t size) const;

    /// @brief Get size required to serialise a message.
    /// @details This function will call getDataSizeImpl() pure virtual
    ///          function. It is a responsibility of the actual message
    ///          to implement it to provide the information.
    /// @return Number of bytes required to serialise a message.
    /// @post Returned value is less than std::numeric_limits<std::size_t>::max();
    /// @note Thread safety: Depends on thread safety of the implementation of
    ///       getDataSizeImpl() provided by the derived class(es).
    /// @note Exception guarantee: No throw.
    std::size_t getDataSize() const;

protected:

    /// @brief Pure virtual function to be called to update contents of the
    ///        message based on the data in the stream buffer.
    /// @details Must be implemented in the derived class
    /// @param[in, out] buf Input stream buffer.
    /// @param[in] size Size of the data in the stream buffer.
    /// @return Status of the read operation.
    /// @note Must comply with all the preconditions, postconditions,
    ///       Thread safety and Exception guarantee specified in read().
    virtual ErrorStatus readImpl(std::streambuf& buf, std::size_t size) = 0;

    /// @brief Pure virtual function to be called to write contents of the
    ///        message to the stream buffer.
    /// @details Must be implemented in the derived class
    /// @param[in, out] buf Output stream buffer.
    /// @param[in] size Maximal size the message data must fit it.
    /// @return Status of the write operation.
    /// @pre getDataSizeImpl() <= size. This check is performed in write()
    ///      prior to call to this function.
    /// @note Must comply with all the preconditions, postconditions,
    ///       Thread safety and Exception guarantee specified in write().
    virtual ErrorStatus writeImpl(std::streambuf& buf, std::size_t size) const = 0;

    /// @brief Pure virtual function to be called to retrieve
    ///        number of bytes required to serialise current message.
    /// @details Must be implemented in the derived class.
    /// @return Number of bytes required to serialise a message.
    /// @note Must comply with all the preconditions, postconditions,
    ///       Thread safety and Exception guarantee specified in getDataSize().
    virtual std::size_t getDataSizeImpl() const = 0;

private:

    /// @brief Implemented in MessageBase
    virtual MsgIdType getIdImpl() const = 0;
};


/// @brief Abstract base class for all the messages.
/// @details This class provides basic API to allow retrieval of message ID,
///          reading and writing of the message contents, as well as dispatching
///          message to its handler.
/// @tparam THandler Handler class that must provide
///         "handleMessage(<message_type>& msg)" member functions to handle
///         all types of defined messages.
/// @tparam TTraits Various behavioural traits relevant for the message.
///         Currently the only trait that is required for this class is
///         Endianness. The traits class/struct must typedef either
///         traits::endian::Big or traits::endian::Little to Endianness.
/// @headerfile embxx/comms/Message.h
template <typename THandler, typename TTraits>
class Message : public CommonMessageBase<>
{
    typedef CommonMessageBase<> Base;
public:

    /// @brief Handler class.
    /// @details Must provide "handleMessage(...)" functions.
    typedef THandler Handler;

    /// @brief Traits class type.
    typedef TTraits Traits;

    /// @brief Type used for message ID
    typedef Base::MsgIdType MsgIdType;

    /// @brief Destructor
    virtual ~Message();

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
    /// Actual Endianness defined in provided Traits class
    typedef typename Traits::Endianness Endianness;

    /// @brief Write data into the output stream buffer.
    /// @details Use this function to write data to the stream buffer.
    ///          The endianness of the data will be as specified in the TTraits
    ///          template parameter of the class.
    /// @tparam T Type of the value to write. Must be integral.
    /// @param[in] value Integral type value to be written.
    /// @param[in, out] buf Output stream buffer.
    /// @return Number of bytes actually written.
    /// @post The internal pointer of the stream buffer is advanced.
    /// @note Thread safety: Safe for distinct buffers, unsafe otherwise.
    /// @note Exception guarantee: Depends on exception safety of the stream
    ///       buffer.
    template <typename T>
    static std::size_t putData(T value, std::streambuf& buf);

    /// @brief Write partial data into the output stream buffer.
    /// @details Use this function to write partial data to the stream buffer.
    ///          The endianness of the data will be as specified in the TTraits
    ///          template parameter of the class.
    /// @tparam TSize Length of the value in bytes known in compile time.
    /// @tparam T Type of the value to write. Must be integral.
    /// @param[in] value Integral type value to be written.
    /// @param[in, out] buf Output stream buffer.
    /// @return Number of bytes actually written.
    /// @pre TSize <= sizeof(T)
    /// @post The internal pointer of the stream buffer is advanced.
    /// @note Thread safety: Safe for distinct buffers, unsafe otherwise.
    /// @note Exception guarantee: Depends on exception safety of the stream
    ///       buffer.
    template <std::size_t TSize, typename T>
    static std::size_t putData(T value, std::streambuf& buf);

    /// @brief Read data from input stream buffer.
    /// @details Use this function to read data from the stream buffer.
    /// The endianness of the data will be as specified in the TTraits
    /// template parameter of the class.
    /// @tparam T Return type
    /// @tparam TSize number of bytes to read
    /// @param[in, out] buf Input stream buffer.
    /// @return The integral type value.
    /// @pre The buffer has required amount of bytes to be read.
    ///      The result is undefined otherwise.
    /// @pre TSize <= sizeof(T)
    /// @post The internal pointer of the stream buffer is advanced.
    /// @note Thread safety: Safe for distinct stream buffers, unsafe otherwise.
    /// @note Exception guarantee: Depends on exception safety of the stream
    ///       buffer.
    template <typename T, std::size_t TSize = sizeof(T)>
    static T getData(std::streambuf& buf);

private:

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

    /// @brief Implements read body behaviour.
    /// @details Does nothing
    /// @return ErrorStatus::Success.
    virtual ErrorStatus readImpl(
        std::streambuf& buf,
        std::size_t size) override final;

    /// @brief Implements read body behaviour.
    /// @details Does nothing
    /// @return ErrorStatus::Success.
    virtual ErrorStatus writeImpl(
        std::streambuf& buf,
        std::size_t size) const override final;

    /// @brief Implements serialisation size retrieval.
    /// @return 0
    virtual std::size_t getDataSizeImpl() const override final;
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
///         @li @code std::size_t getLength() const; @endcode Provides an
///             information about length of the field when serialised. May also
///             be static if doesn't depen on the field value.
///         @li @code ErrorStatus read(std::streambuf& buf, std::size_t size); @endcode
///             Reads the serialised value from the input stream buffer and
///             updates its value.
///         @li @code ErrorStatus write(std::streambuf& buf, std::size_t size) const; @endcode
///             Serialises value into the output stream buffer.
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
    MetaMessageBase();

    /// @brief Destructor
    virtual ~MetaMessageBase();

    Fields& getFields();
    const Fields& getFields() const;

protected:
    /// @brief Implements read body behaviour.
    /// @details Calls read() member function of every element in TFields.
    /// @return ErrorStatus::Success if all read operations are successful.
    virtual ErrorStatus readImpl(
        std::streambuf& buf,
        std::size_t size) override;

    /// @brief Implements write body behaviour.
    /// @details Calls write() member function of every element in TField
    /// @return ErrorStatus::Success if all write operations are successful.
    virtual ErrorStatus writeImpl(
        std::streambuf& buf,
        std::size_t size) const override;

    /// @brief Implements serialisation size retrieval.
    /// @details Calls getLength() member functio nof every element it TField
    ///          and sums the result
    /// @return Number of bytes required to serialise all fields in TField
    virtual std::size_t getDataSizeImpl() const override;

private:

    class FieldReader
    {
    public:
        FieldReader(std::streambuf& buf, ErrorStatus& status, std::size_t& size)
            : buf_(buf),
              status_(status),
              size_(size)
        {

        }

        template <typename TField>
        void operator()(TField& field) {
            if (status_ == ErrorStatus::Success) {
                status_ = field.read(buf_, size_);
                if (status_ == ErrorStatus::Success) {
                    GASSERT(field.getLength() <= size_);
                    size_ -= field.getLength();
                }
            }
        }
    private:
        std::streambuf& buf_;
        ErrorStatus status_;
        std::size_t& size_;
    };

    class FieldWriter
    {
    public:
        FieldWriter(std::streambuf& buf, ErrorStatus& status, std::size_t& size)
            : buf_(buf),
              status_(status),
              size_(size)
        {

        }

        template <typename TField>
        void operator()(TField& field) {
            if (status_ == ErrorStatus::Success) {
                status_ = field.write(buf_, size_);
                if (status_ == ErrorStatus::Success) {
                    GASSERT(field.getLength() <= size_);
                    size_ -= field.getLength();
                }
            }
        }

    private:
        std::streambuf& buf_;
        ErrorStatus status_;
        std::size_t& size_;
    };


    struct FieldLengthRetriever
    {
        template <typename TField>
        std::size_t operator()(std::size_t size, const TField& field)
        {
            return size + field.getLength();
        }
    };

    Fields fields_;
};


/// @}

// Implementation
template <typename T>
CommonMessageBase<T>::~CommonMessageBase()
{
}

template <typename T>
typename CommonMessageBase<T>::MsgIdType
CommonMessageBase<T>::getId() const
{
    // redirecting request to derived class
    return this->getIdImpl();
}

template <typename T>
ErrorStatus CommonMessageBase<T>::read(
    std::streambuf& buf,
    std::size_t size)
{
    GASSERT(size <= static_cast<decltype(size)>(buf.in_avail()));

    auto minSize = getDataSize();
    ErrorStatus status = ErrorStatus::NumOfErrorStatuses;
    do {
        if (size < minSize) {
            status = ErrorStatus::NotEnoughData;
            break;
        }

        status = this->readImpl(buf, size);
    } while (false);

    return status;
}

template <typename T>
ErrorStatus CommonMessageBase<T>::write(
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

    ErrorStatus status = ErrorStatus::NumOfErrorStatuses;
    do {
        if (size < getDataSize()) {
            status = ErrorStatus::BufferOverflow;
            break;
        }

        status = this->writeImpl(buf, size);
    } while (false);

    return status;
}

template <typename T>
std::size_t CommonMessageBase<T>::getDataSize() const
{
    auto value = this->getDataSizeImpl();
    GASSERT(value < std::numeric_limits<decltype(value)>::max());
    return value;
}


template <typename THandler, typename TTraits>
Message<THandler, TTraits>::~Message()
{
}

template <typename THandler, typename TTraits>
void Message<THandler, TTraits>::dispatch(Handler& handler)
{
    this->dispatchImpl(handler);
}

template <typename THandler, typename TTraits>
template <typename T>
std::size_t Message<THandler, TTraits>::putData(T value, std::streambuf& buf)
{
    return io::putData<T>(value, buf, Endianness());
}

template <typename THandler, typename TTraits>
template <std::size_t TSize, typename T>
std::size_t Message<THandler, TTraits>::putData(T value, std::streambuf& buf)
{
    static_assert(TSize <= sizeof(T),
                                "Cannot put more bytes than type contains");
    return io::putData<TSize, T>(value, buf, Endianness());
}

template <typename THandler, typename TTraits>
template <typename T, std::size_t TSize>
T Message<THandler, TTraits>::getData(std::streambuf& buf)
{
    static_assert(TSize <= sizeof(T),
                                "Cannot get more bytes than type contains");
    return io::getData<T, TSize>(buf, Endianness());
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
    std::streambuf& buf,
    std::size_t size)
{
    static_cast<void>(buf);
    static_cast<void>(size);
    return ErrorStatus::Success;
}

template <typename traits::MsgIdType TId,
          typename TBase,
          typename TActual>
ErrorStatus EmptyBodyMessage<TId, TBase, TActual>::writeImpl(
    std::streambuf& buf,
    std::size_t size) const
{
    static_cast<void>(buf);
    static_cast<void>(size);
    return ErrorStatus::Success;
}

template <typename traits::MsgIdType TId,
          typename TBase,
          typename TActual>
std::size_t EmptyBodyMessage<TId, TBase, TActual>::getDataSizeImpl() const
{
    return 0;
}

template <traits::MsgIdType TId,
          typename TBase,
          typename TActual,
          typename TFields>
MetaMessageBase<TId, TBase, TActual, TFields>::MetaMessageBase()
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
    std::streambuf& buf,
    std::size_t size)
{
    GASSERT(size <= buf.in_avail());
    ErrorStatus status = ErrorStatus::Success;
    std::size_t remainingSize = size;
    util::tupleForEach(fields_, FieldReader(buf, status, remainingSize));
    return status;
}

template <traits::MsgIdType TId,
          typename TBase,
          typename TActual,
          typename TFields>
ErrorStatus MetaMessageBase<TId, TBase, TActual, TFields>::writeImpl(
    std::streambuf& buf,
    std::size_t size) const
{
    ErrorStatus status = ErrorStatus::Success;
    std::size_t remainingSize = size;
    util::tupleForEach(fields_, FieldWriter(buf, status, remainingSize));
    return status;
}

template <traits::MsgIdType TId,
          typename TBase,
          typename TActual,
          typename TFields>
std::size_t MetaMessageBase<TId, TBase, TActual, TFields>::getDataSizeImpl() const
{
    return util::tupleAccumulate(fields_, 0U, FieldLengthRetriever());
}


}  // namespace comms

}  // namespace embxx
