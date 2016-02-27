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

/// @file embxx/comms/protocol/MsgIdLayer.h
/// This file contains "Message ID" protocol layer of the "comms" module.

#pragma once

#include <array>
#include <tuple>
#include <algorithm>
#include <utility>

#include "embxx/util/Assert.h"
#include "embxx/util/Tuple.h"
#include "embxx/comms/traits.h"
#include "ProtocolLayer.h"

namespace embxx
{

namespace comms
{

namespace protocol
{

/// @ingroup comms
/// @brief Protocol layer that uses message ID to differentiate between messages.
/// @details This layers is a "must have" one, it contains allocator to allocate
///          message object.
/// @tparam TAllMessages A tuple (std::tuple) of all the custom message types
///         this protocol layer must support. The messages in the tuple must
///         be sorted in ascending order based on their MsgId
/// @tparam TAllocator The allocator class, will be used to allocate message
///         objects in read() member function.
///         The requirements for the allocator are:
///         @li Must have a default (no arguments) constructor.
///         @li Must provide allocation function to allocate message
///             objects. The signature must be as following:
///             @code template <typename TObj, typename... TArgs> std::unique_ptr<TObj, Deleter> alloc(TArgs&&... args); @endcode
///             The Deleter maybe either default "std::default_delete<T>" or
///             custom one. All the allocators defined in "util" module
///             (header: "embxx/util/Allocators.h") satisfy these requirements.
///             See also embxx::comms::DynMemMsgAllocator and
///             embxx::comms::InPlaceMsgAllocator
/// @tparam TTraits A traits class that must define:
///         @li Endianness type. Either embxx::comms::traits::endian::Big or
///             embxx::comms::traits::endian::Little
///         @li MsgIdLen static integral constant specifying length of
///             message ID field in bytes.
/// @pre TAllMessages must be any variation of std::tuple
/// @pre All message types in TAllMessages must be in ascending order based on
///      their MsgId value
/// @headerfile embxx/comms/protocol/MsgIdLayer.h
template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
class MsgIdLayer : public ProtocolLayer<TTraits, TNextLayer>
{
    static_assert(util::IsTuple<TAllMessages>::Value,
        "TAllMessages must be of std::tuple type");
    typedef ProtocolLayer<TTraits, TNextLayer> Base;

    typedef decltype(TAllocator().template alloc<typename Base::MsgBase>()) InternalMsgPtr;

public:

    /// @brief Base class for all the messages
    typedef typename Base::MsgBase MsgBase;

    /// @brief Definition of all messages type. Must be std::tuple
    typedef TAllMessages AllMessages;

    /// @brief Definition of the allocator type
    typedef TAllocator Allocator;

    /// @brief Used traits
    typedef typename Base::Traits Traits;

    /// @brief Smart pointer to the created message.
    /// @details Equivalent to:
    ///          @code
    ///          typedef decltype(Allocator().template alloc<MsgBase>()) MsgPtr;
    ///          @endcode
    typedef InternalMsgPtr MsgPtr;

    /// @brief Type of message ID
    typedef traits::MsgIdType MsgIdType;

    /// @brief Type of read iterator
    typedef typename Base::ReadIterator ReadIterator;

    /// @brief Type of write iterator
    typedef typename Base::WriteIterator WriteIterator;

    /// Length of the message ID field. Originally defined in traits
    static const std::size_t MsgIdLen = Traits::MsgIdLen;

    /// @brief Constructor
    /// @details Defines static factories responsible for generation of
    ///          custom message objects.
    /// @tparam Types of parameters
    /// @param args Parameters, all forwarded to the next layer
    /// @note Thread safety: Safe if compiler supports safe initialisation
    ///       of static data
    /// @note Exception guarantee: Basic
    template <typename... TArgs>
    explicit MsgIdLayer(TArgs&&... args);

    /// @brief Copy constructor is default
    MsgIdLayer(const MsgIdLayer&) = default;

    /// @brief Move constructor is default
    MsgIdLayer(MsgIdLayer&&) = default;

    /// @brief Copy assignment is default
    MsgIdLayer& operator=(const MsgIdLayer&) = default;

    /// @brief Move assignment is default.
    MsgIdLayer& operator=(MsgIdLayer&&) = default;

    /// @brief Destructor
    ~MsgIdLayer();

    /// @brief Deserialise message from the input data sequence.
    /// @details The function will read message ID from the data sequence first,
    ///          generate appropriate message object based on the read ID and
    ///          forward the request to the next layer.
    /// @param[in, out] msgPtr Reference to smart pointer that will hold
    ///                 allocated message object
    /// @param[in, out] iter Input iterator
    /// @param[in] size Size of the data in the sequence
    /// @param[out] missingSize If not nullptr and return value is
    ///             embxx::comms::ErrorStatus::NotEnoughData it will contain
    ///             minimal missing data length required for the successful
    ///             read attempt.
    /// @return Error status of the operation.
    /// @pre msgPtr doesn't point to any object:
    ///      @code assert(!msgPtr); @endcode
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "size" times;
    /// @post The iterator will be advanced by the number of bytes was actually
    ///       read. In case of an error, distance between original position and
    ///       advanced will pinpoint the location of the error.
    /// @post Returns embxx::comms::ErrorStatus::Success if and only if msgPtr points
    ///       to a valid object.
    /// @post missingSize output value is updated if and only if function
    ///       returns embxx::comms::ErrorStatus::NotEnoughData.
    /// @note Thread safety: Safe on distinct MsgIdLayer object and distinct
    ///       buffers, unsafe otherwise.
    /// @note Exception guarantee: Basic
    ErrorStatus read(
        MsgPtr& msgPtr,
        ReadIterator& iter,
        std::size_t size,
        std::size_t* missingSize = nullptr);

    /// @brief Serialise message into output data sequence.
    /// @details The function will write ID of the message to the data
    ///          sequence, then call write() member function of the next
    ///          protocol layer.
    /// @param[in] msg Reference to message object
    /// @param[in, out] iter Output iterator.
    /// @param[in] size Available space in data sequence.
    /// @return Status of the write operation.
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "size" times;
    /// @post The iterator will be advanced by the number of bytes was actually
    ///       written. In case of an error, distance between original position
    ///       and advanced will pinpoint the location of the error.
    /// @note Thread safety: Safe on distinct stream buffers, unsafe otherwise.
    /// @note Exception guarantee: Basic
    ErrorStatus write(
        const MsgBase& msg,
        WriteIterator& iter,
        std::size_t size) const;

    /// @brief Update the recently written output data sequence.
    /// @details In case write() operation returned
    ///          embxx::comms::ErrorStatus::UpdateRequired the output operator
    ///          wasn't of random access type, such as std::back_insert_iterator
    ///          for example. In this case some protocol fields may contain
    ///          invalid data that needs to be updated. This function is used
    ///          for this purpose.
    /// @tparam TUpdateIter Type of update iterator
    /// @param iter Random access iterator used to both read and update
    ///        already written data.
    /// @param size Size of the written data.
    /// @return Status of the update operation.
    /// @pre Iterator must be of random access type.
    /// @post The iterator is advanced to the end of current protocol
    ///       layer data.
    template <typename TUpdateIter>
    ErrorStatus update(
        TUpdateIter& iter,
        std::size_t size) const;

    /// @brief Returns minimal protocol stack length required to serialise
    ///        empty message.
    /// @details Adds "MsgIdLen" to the result of nextLayer().length().
    constexpr std::size_t length() const;

    /// @brief Returns message serialisation length including protocol stack
    ///        overhead.
    /// @details Adds "MsgIdLen" to the result of nextLayer().length(msg).
    std::size_t length(const MsgBase& msg) const;

    /// @brief Get allocator.
    /// @details Returns reference to the message allocator. It can be used
    ///          to extend initialisation of the latter if its default
    ///          constructor wasn't enough.
    /// @return Reference to message allocator
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    Allocator& getAllocator();

    /// @brief Const version of getAllocator()
    const Allocator& getAllocator() const;

private:

    /// @cond DOCUMENT_MSG_ID_PROTOCOL_LAYER_FACTORY
    class Factory
    {
    public:

        virtual ~Factory();

        MsgIdType getId() const;

        MsgPtr create(Allocator& allocator) const;

    protected:
        virtual MsgIdType getIdImpl() const = 0;
        virtual MsgPtr createImpl(Allocator& allocator) const = 0;
    };

    template <typename TMessage>
    class MsgFactory : public Factory
    {
    public:
        typedef TMessage Message;
        static const MsgIdType MsgId = Message::MsgId;
    protected:
        virtual MsgIdType getIdImpl() const;
        virtual MsgPtr createImpl(Allocator& allocator) const;
    private:
    };
    /// @endcond

    typedef std::array<Factory*, std::tuple_size<AllMessages>::value> Factories;

    Allocator allocator_;
    Factories factories_;

};

// Implementation

namespace details
{

template <std::size_t TSize>
struct FactoryCreator
{
    template <typename TAllMessages,
              template <class> class TFactory,
              typename TFactories>
    static void create(TFactories& factories)
    {
        static const std::size_t Idx = TSize - 1;
        FactoryCreator<Idx>::template
                            create<TAllMessages, TFactory>(factories);

        typedef typename std::tuple_element<Idx, TAllMessages>::type Message;
        static TFactory<Message> factory;
        factories[Idx] = &factory;
    }
};

template <>
struct FactoryCreator<0>
{
    template <typename TAllMessages,
              template <class> class TFactory,
              typename TFactories>
    static void create(TFactories& factories)
    {
        static_cast<void>(factories);
    }
};

template <std::size_t TEndIdx, typename TAllMessages>
struct AreMessagesSorted
{
    typedef typename std::tuple_element<TEndIdx - 2, TAllMessages>::type FirstElemType;
    typedef typename std::tuple_element<TEndIdx - 1, TAllMessages>::type SecondElemType;

    static const bool Value =
        ((FirstElemType::MsgId < SecondElemType::MsgId) &&
         (AreMessagesSorted<TEndIdx - 1, TAllMessages>::Value));
};

template <typename TAllMessages>
struct AreMessagesSorted<1, TAllMessages>
{
    static const bool Value = true;
};

template <typename TAllMessages>
struct AreMessagesSorted<0, TAllMessages>
{
    static const bool Value = true;
};




}  // namespace details


template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
template <typename... TArgs>
MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::MsgIdLayer(
    TArgs&&... args)
    : Base(std::forward<TArgs>(args)...)
{
    static const std::size_t NumOfMsgs = std::tuple_size<AllMessages>::value;

    static_assert(details::AreMessagesSorted<NumOfMsgs, AllMessages>::Value,
        "All the message types in the bundle must be sorted in ascending order "
        "based on their MsgId");

    details::FactoryCreator<NumOfMsgs>::template
                    create<AllMessages, MsgFactory>(factories_);
}

template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::~MsgIdLayer()
{
}

template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
ErrorStatus MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::read(
    MsgPtr& msgPtr,
    ReadIterator& iter,
    std::size_t size,
    std::size_t* missingSize)
{
    GASSERT(!msgPtr);
    if (size < MsgIdLen) {
        if (missingSize != nullptr) {
            *missingSize = length() - size;
        }
        return ErrorStatus::NotEnoughData;
    }

    auto id = Base::template readData<MsgIdType, MsgIdLen>(iter);
    auto factoryIter = std::lower_bound(factories_.begin(), factories_.end(), id,
        [](Factory* factory, MsgIdType idVal) -> bool
        {
            return factory->getId() < idVal;
        });

    if ((factoryIter == factories_.end()) || ((*factoryIter)->getId() != id)) {
        return ErrorStatus::InvalidMsgId;
    }

    msgPtr = (*factoryIter)->create(allocator_);
    if (!msgPtr) {
        return ErrorStatus::MsgAllocFaulure;
    }

    auto status = Base::nextLayer().read(msgPtr, iter, size - MsgIdLen, missingSize);
    if (status != ErrorStatus::Success) {
        msgPtr.reset();
    }

    return status;
}

template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
ErrorStatus MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::write(
    const MsgBase& msg,
    WriteIterator& iter,
    std::size_t size) const
{
    if (size < MsgIdLen) {
        return ErrorStatus::BufferOverflow;
    }

    Base::template writeData<MsgIdLen>(msg.getId(), iter);
    return Base::nextLayer().write(msg, iter, size - MsgIdLen);
}

template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
template <typename TUpdateIter>
ErrorStatus MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::update(
    TUpdateIter& iter,
    std::size_t size) const
{
    std::advance(iter, MsgIdLen);
    return Base::nextLayer().update(iter, size - MsgIdLen);
}

template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
constexpr
std::size_t MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::length() const
{
    return MsgIdLen + Base::nextLayer().length();
}

template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
std::size_t MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::length(
    const MsgBase& msg) const
{
    return MsgIdLen + Base::nextLayer().length(msg);
}

template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
typename MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::Allocator&
MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::getAllocator()
{
    return allocator_;
}

template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
const typename MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::Allocator&
MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::getAllocator() const
{
    return allocator_;
}

/// @cond DOCUMENT_MSG_ID_PROTOCOL_LAYER_FACTORY
template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::Factory::~Factory()
{
}


template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
typename MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::MsgIdType
MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::Factory::getId() const
{
    return this->getIdImpl();
}

template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
typename MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::MsgPtr
MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::Factory::create(
    Allocator& allocator) const
{
    return this->createImpl(allocator);
}

template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
template <typename TMessage>
typename MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::MsgIdType
MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::MsgFactory<TMessage>::getIdImpl() const
{
    return MsgId;
}

template <typename TAllMessages,
          typename TAllocator,
          typename TTraits,
          typename TNextLayer>
template <typename TMessage>
typename MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::MsgPtr
MsgIdLayer<TAllMessages, TAllocator, TTraits, TNextLayer>::MsgFactory<TMessage>::createImpl(
    Allocator& allocator) const
{
    return std::move(allocator.template alloc<Message>());
}

/// @endcond

}  // namespace protocol

}  // namespace comms

}  // namespace embxx
