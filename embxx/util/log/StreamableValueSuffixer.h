//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file util/log/StreamableValueSuffixer.h
/// Contains definition of StreamableValueSuffixer decorator of the StreamLogger.

#pragma once

#include "StreamableValueDecoratorBase.h"

namespace embxx
{

namespace util
{

namespace log
{

/// @addtogroup util
/// @{

/// @brief Defines a decorator that redirects provided value to the output
///        stream afther the call to end() member function of the next layer.
/// @headerfile embxx/util/log/StreamableValueSuffixer.h
template <typename T, typename TNextLayer>
class StreamableValueSuffixer : public StreamableValueDecoratorBase<T, TNextLayer>
{
    typedef StreamableValueDecoratorBase<T, TNextLayer> Base;
public:

    /// Redefinition of the Value Type
    typedef typename Base::ValueType ValueType;

    /// @brief Constructor
    /// @details Keeps a copy of the provided streamable value and forwards all
    ///          the other parameters to the constructor of the next layer.
    /// @param[in] value streamable value
    /// @param[in] params Zero or more parameters to be forwarded to the next layer.
    /// @note Exception guarantee: Strong
    template<typename... TParams>
    StreamableValueSuffixer(ValueType&& value, TParams&&... params);

    /// @brief Redirects provided streamable value to the output stream after
    ///        call to the end() member function of the next layer.
    /// @param[in] level requested logging level
    /// @note Exception guarantee: Strong
    void end(Level level);
};

/// @}

// Implementation
template <typename T, typename TNextLayer>
template<typename... TParams>
StreamableValueSuffixer<T, TNextLayer>::StreamableValueSuffixer(
    ValueType&& value,
    TParams&&... params)
    : Base(std::forward<ValueType>(value), std::forward<TParams>(params)...)
{
}

template <typename T, typename TNextLayer>
void StreamableValueSuffixer<T, TNextLayer>::end(
    Level level)
{
    Base::end(level);
    Base::stream() << Base::value_;
}

}  // namespace log

}  // namespace util

}  // namespace embxx
