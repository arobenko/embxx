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


#pragma once


#include "ErrorCode.h"

namespace embxx
{

namespace error
{

/// @addtogroup error
/// @{

/// @brief Error code wrapper class.
/// @details This class wraps error code enum value to provide simple boolean
/// checks for operation success/error
/// @tparam TErrorCode Error code enum type. 0 value must correspond to success.
/// @headerfile embxx/error/ErrorStatus.h
template <typename TErrorCode = ErrorCode>
class ErrorStatusT
{
public:
    /// @brief Error code enum type
    typedef TErrorCode ErrorCodeType;

    /// @brief Default constructor.
    /// @details The code value is 0, which is "success".
    ErrorStatusT();

    /// @brief Constructor
    /// @details This constructor may be used for implicit construction of
    ///          error status object out of error code value.
    /// @param code Numeric error code value.
    ErrorStatusT(ErrorCodeType code);

    /// @brief Copy constructor is default
    ErrorStatusT(const ErrorStatusT&) = default;

    /// @brief Destructor is default
    ~ErrorStatusT() = default;

    /// @brief Copy assignment is default
    ErrorStatusT& operator=(const ErrorStatusT&) = default;

    /// @brief Retrieve error code value.
    const ErrorCodeType code() const;

    /// @brief boolean conversion operator.
    /// @details Returns true if error code is not equal 0, i.e. any error
    ///          will return true, success value will return false.
    operator bool() const;

    /// @brief Same as !(static_cast<bool>(*this)).
    bool operator!() const;

private:
    ErrorCodeType code_;
};

/// @brief ErrorStatus that uses embxx::error::ErrorCode for its error codes.
/// @related ErrorStatusT
/// @headerfile embxx/error/ErrorStatus.h
typedef ErrorStatusT<ErrorCode> ErrorStatus;

// Implementation
/// @brief Error status equality comparison operator
/// @related ErrorStatusT
template <typename TErrorCode>
bool operator==(
    const ErrorStatusT<TErrorCode>& es1,
    const ErrorStatusT<TErrorCode>& es2)
{
    return es1.value() == es2.value();
}

/// @brief Error status non-equality comparison operator
/// @related ErrorStatusT
template <typename TErrorCode>
bool operator!=(
    const ErrorStatusT<TErrorCode>& es1,
    const ErrorStatusT<TErrorCode>& es2)
{
    return !(es1 == es2);
}

/// @}

template <typename TErrorCode>
ErrorStatusT<TErrorCode>::ErrorStatusT()
    : code_(static_cast<ErrorCodeType>(0))
{
}

template <typename TErrorCode>
ErrorStatusT<TErrorCode>::ErrorStatusT(ErrorCodeType code)
    : code_(code)
{
}

template <typename TErrorCode>
const typename ErrorStatusT<TErrorCode>::ErrorCodeType
ErrorStatusT<TErrorCode>::code() const
{
    return code_;
}

template <typename TErrorCode>
ErrorStatusT<TErrorCode>::operator bool() const
{
    return (code_ != static_cast<ErrorCodeType>(0));
}

template <typename TErrorCode>
bool ErrorStatusT<TErrorCode>::operator!() const
{
    return !(static_cast<bool>(*this));
}

}  // namespace error

}  // namespace embxx

