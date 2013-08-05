//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file util/log/StreamableValueDecoratorBase.h
/// Contains definition of StreamableValueDecoratorBase.

#pragma once

#include "LoggerDecoratorBase.h"

namespace embxx
{

namespace util
{

namespace log
{

/// @addtogroup util
/// @{

/// @brief Base class for the StreamableValuePrefixer and
///        StreamableValueSuffixer
/// @details It contains common functionality for both dirived classes
/// @tparam T Streamable type
/// @tparam TNextLayer Next layer type
/// @headerfile embxx/util/log/StreamableValueDecoratorBase.h
template <typename T, typename TNextLayer>
class StreamableValueDecoratorBase : public LoggerDecoratorBase<TNextLayer>
{
    typedef LoggerDecoratorBase<TNextLayer> Base;
public:
    /// Type of the streamable value
    typedef T ValueType;

    /// @brief Constructor
    /// @details Keeps a copy of the provided streamable value and forwards all
    ///          the other parameters to the constructor of the next layer.
    /// @param[in] value streamable value
    /// @param[in] params Zero or more parameters to be forwarded to the next layer.
    /// @note Exception guarantee: Strong
    template<typename... TParams>
    StreamableValueDecoratorBase(ValueType&& value, TParams&&... params);

protected:
    ValueType value_; ///< Streamable value
};

/// @}

// Implementation
template <typename T, typename TNextLayer>
template<typename... TParams>
StreamableValueDecoratorBase<T, TNextLayer>::StreamableValueDecoratorBase(
    ValueType&& value,
    TParams&&... params)
    : Base(std::forward<TParams>(params)...),
      value_(std::forward<ValueType>(value))
{
}

}  // namespace log

}  // namespace util

}  // namespace embxx
