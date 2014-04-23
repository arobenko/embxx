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

/// @file embxx/comms/protocol/ProtocolLayer.h
/// This file contains definition of the base class fro all protocol layers
/// of the "comms" module.

#pragma once

#include <cstddef>
#include "embxx/io/access.h"
#include "embxx/comms/traits.h"

namespace embxx
{

namespace comms
{

namespace protocol
{

/// @addtogroup comms
/// @{

/// @brief Base class for all the mid level protocol layers.
/// @details Defines common types and operations for all mid level
///          protocol layers.
/// @tparam TTraits Traits class with all the necessary types and constants.
///         Requires definition of Endianness type, which can be either
///         embxx::comms::traits::endian::Big or embxx::comms::traits::endian::Little.
/// @tparam TNextLayer Next layer in the protocol stack. The next layer type
///         must define the following:
///         @li MsgPtr type - smart pointer to message object
///         @li MsgBase type - base class to all the custom messages
///         @li ReadIterator type - input iterator type
///         @li WriteIterator type - output iterator type
/// @headerfile embxx/comms/protocol/ProtocolLayer.h
template <typename TTraits, typename TNextLayer>
class ProtocolLayer
{

public:
    /// @brief Next layer type
    typedef TNextLayer NextLayer;

    /// @brief Pointer to message object
    typedef typename NextLayer::MsgPtr MsgPtr;

    /// Base class to all custom messages
    typedef typename NextLayer::MsgBase MsgBase;

    /// @brief Read iterator type
    typedef typename NextLayer::ReadIterator ReadIterator;

    /// @brief Write iterator type
    typedef typename NextLayer::WriteIterator WriteIterator;

protected:

    /// @brief Traits type
    typedef TTraits Traits;

    /// @brief Endianness type
    typedef typename Traits::Endianness Endianness;

    /// @brief Default constructor
    template <typename... TArgs>
    explicit ProtocolLayer(TArgs&&... args);

    /// @brief Copy constructor is default
    ProtocolLayer(const ProtocolLayer&) = default;

    /// @brief Move constructor is default
    ProtocolLayer(ProtocolLayer&&) = default;

    /// @brief Copy assignment is default
    ProtocolLayer& operator=(const ProtocolLayer&) = default;

    /// @brief Move assignment is default
    ProtocolLayer& operator=(ProtocolLayer&&) = default;

    /// @brief Retrieve reference to the next layer object.
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    NextLayer& nextLayer();

    /// @brief Const version of nextLayer()
    const NextLayer& nextLayer() const;

    /// @brief Write data into the output data sequence.
    /// @details Use this function to write data to the output data sequence.
    ///          The endianness of the data will be as specified in the TTraits
    ///          template parameter of the class.
    /// @tparam T Type of the value to write. Must be integral.
    /// @tparam TIter Type of output iterator
    /// @param[in] value Integral type value to be written.
    /// @param[in, out] iter Output iterator.
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "sizeof(T)" times;
    /// @post The iterator is advanced.
    /// @note Thread safety: Safe for distinct buffers, unsafe otherwise.
    template <typename T, typename TIter>
    static void writeData(T value, TIter& iter);

    /// @brief Write partial data into the output data sequence.
    /// @details Use this function to write partial data to the output data sequence.
    ///          The endianness of the data will be as specified in the TTraits
    ///          template parameter of the class.
    /// @tparam TSize Length of the value in bytes known in compile time.
    /// @tparam T Type of the value to write. Must be integral.
    /// @tparam TIter Type of the output iterator
    /// @param[in] value Integral type value to be written.
    /// @param[in, out] iter Output iterator.
    /// @pre TSize <= sizeof(T)
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "TSize" times;
    /// @post The iterator is advanced.
    /// @note Thread safety: Safe for distinct buffers, unsafe otherwise.
    template <std::size_t TSize, typename T, typename TIter>
    static void writeData(T value, TIter& iter);

    /// @brief Read data from input data sequence.
    /// @details Use this function to read data from input data sequence.
    /// The endianness of the data will be as specified in the TTraits
    /// template parameter of the class.
    /// @tparam T Return type
    /// @tparam TIter Type of input iterator.
    /// @param[in, out] iter Input iterator.
    /// @return The integral type value.
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "sizeof(T)" times;
    /// @post The iterator is advanced.
    /// @note Thread safety: Safe for distinct buffers, unsafe otherwise.
    template <typename T, typename TIter>
    static T readData(TIter& iter);

    /// @brief Read partial data from input data sequence.
    /// @details Use this function to read partial data from input data sequence.
    /// The endianness of the data will be as specified in the TTraits
    /// template parameter of the class.
    /// @tparam T Return type
    /// @tparam TSize number of bytes to read
    /// @tparam TIter Type of input iterator.
    /// @param[in, out] iter Input iterator.
    /// @return The integral type value.
    /// @pre TSize <= sizeof(T)
    /// @pre Iterator must be valid and can be dereferenced and incremented at
    ///      least "TSize" times;
    /// @post The iterator is advanced.
    /// @note Thread safety: Safe for distinct buffers, unsafe otherwise.
    template <typename T, std::size_t TSize, typename TIter>
    static T readData(TIter& iter);

private:
    NextLayer nextLayer_;
};

/// @}

// Implementation
template <typename TTraits, typename TNextLayer>
template <typename... TArgs>
ProtocolLayer<TTraits, TNextLayer>::ProtocolLayer(TArgs&&... args)
    : nextLayer_(std::forward<TArgs>(args)...)
{
}

template <typename TTraits, typename TNextLayer>
typename ProtocolLayer<TTraits, TNextLayer>::NextLayer&
ProtocolLayer<TTraits, TNextLayer>::nextLayer()
{
    return nextLayer_;
}

template <typename TTraits, typename TNextLayer>
const typename ProtocolLayer<TTraits, TNextLayer>::NextLayer&
ProtocolLayer<TTraits, TNextLayer>::nextLayer() const
{
    return nextLayer_;
}

template <typename TTraits, typename TNextLayer>
template <typename T, typename TIter>
void ProtocolLayer<TTraits, TNextLayer>::writeData(
    T value,
    TIter& iter)
{
    writeData<sizeof(T)>(value, iter);
}


template <typename TTraits, typename TNextLayer>
template <std::size_t TSize, typename T, typename TIter>
void ProtocolLayer<TTraits, TNextLayer>::writeData(
    T value,
    TIter& iter)
{
    static_assert(TSize <= sizeof(T),
        "Cannot write more bytes than type contains");

    io::writeData<TSize, T>(value, iter, Endianness());
}

template <typename TTraits, typename TNextLayer>
template <typename T, typename TIter>
T ProtocolLayer<TTraits, TNextLayer>::readData(TIter& iter)
{
    return readData<T, sizeof(T)>(iter);
}

template <typename TTraits, typename TNextLayer>
template <typename T, std::size_t TSize, typename TIter>
T ProtocolLayer<TTraits, TNextLayer>::readData(TIter& iter)
{
    static_assert(TSize <= sizeof(T),
        "Cannot read more bytes than type contains");
    return io::readData<T, TSize>(iter, Endianness());
}


}  // namespace protocol

}  // namespace comms

}  // namespace embxx
