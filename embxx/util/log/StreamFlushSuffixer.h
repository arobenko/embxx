//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file util/log/StreamFlushSuffixer.h
/// Contains definition of StreamFlushSuffixer decorator of the StreamLogger.

#pragma once

#include <iostream>
#include "LoggerDecoratorBase.h"

namespace embxx
{

namespace util
{

namespace log
{

/// @addtogroup util
/// @{

/// @brief Defines a decorator that calls flush() method of the
///        provided stream after the call to the end() member function
///        of the next layer.
/// @headerfile embxx/util/log/StreamFlushSuffixer.h
template <typename TNextLayer>
class StreamFlushSuffixer : public LoggerDecoratorBase<TNextLayer>
{
    typedef LoggerDecoratorBase<TNextLayer> Base;
public:

    /// @brief Constructor
    /// @details Forwards all the other parameters to the constructor of
    ///          the next layer.
    /// @param[in] params Zero or more parameters to be forwarded to the next layer.
    /// @note Exception guarantee: Strong
    template<typename... TParams>
    StreamFlushSuffixer(TParams&&... params);

    /// @brief Calls "flush()" method of the output stream after calling
    ///        end() member function of the next layer.
    /// @param[in] level requested logging level
    /// @note Exception guarantee: Strong
    void end(Level level);
};

/// @}

// Implementation
template <typename TNextLayer>
template<typename... TParams>
StreamFlushSuffixer<TNextLayer>::StreamFlushSuffixer(
    TParams&&... params)
    : Base(std::forward<TParams>(params)...)
{
}

template <typename TNextLayer>
void StreamFlushSuffixer<TNextLayer>::end(
    Level level)
{
    Base::end(level);
    Base::stream().flush();
}

}  // namespace log

}  // namespace util

}  // namespace embxx

