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

#include "embxx/io/std_streambuf_access.h"
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
template <typename TTraits, typename TNextLayer>
class ProtocolLayer
{

public:
    /// Next layer type
    typedef TNextLayer NextLayer;

    /// Pointer to message object
    typedef typename NextLayer::MsgPtr MsgPtr;

protected:

    /// Base class to all custom messages
    typedef typename NextLayer::MsgBase MsgBase;

    /// Constructor
    ProtocolLayer() = default;

    /// @brief Retrieve reference to the next layer object.
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    NextLayer& nextLayer();

    /// @brief Const version of nextLayer()
    const NextLayer& nextLayer() const;

    /// @brief Traits type
    typedef TTraits Traits;

    /// @brief Endianness type
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
    NextLayer nextLayer_;
};

/// @}

// Implementation
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
template <typename T>
std::size_t ProtocolLayer<TTraits, TNextLayer>::putData(
    T value,
    std::streambuf& buf)
{
    return io::putData<T>(value, buf, Endianness());
}


template <typename TTraits, typename TNextLayer>
template <std::size_t TSize, typename T>
std::size_t ProtocolLayer<TTraits, TNextLayer>::putData(
    T value,
    std::streambuf& buf)
{
    static_assert(TSize <= sizeof(T),
                                "Cannot put more bytes than type contains");

    return io::putData<TSize, T>(value, buf, Endianness());
}

template <typename TTraits, typename TNextLayer>
template <typename T, std::size_t TSize>
T ProtocolLayer<TTraits, TNextLayer>::getData(std::streambuf& buf)
{
    static_assert(TSize <= sizeof(T),
                                "Cannot get more bytes than type contains");
    return io::getData<T, TSize>(buf, Endianness());
}


}  // namespace protocol

}  // namespace comms

}  // namespace embxx
