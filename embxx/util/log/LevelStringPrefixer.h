//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file util/log/LevelStringPrefixer.h
/// Contains definition of LevelStringPrefixer.

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

/// @brief Defines decorator that prints log level before the requested
///        output
/// @details The output will be one of the following strings:
///          @li [DEBUG]
///          @li [TRACE]
///          @li [INFO]
///          @li [WARNING]
///          @li [ERROR]
/// @tparam TNextLayer Next layer type
/// @headerfile embxx/util/log/LevelStringPrefixer.h
template <typename TNextLayer>
class LevelStringPrefixer : public LoggerDecoratorBase<TNextLayer>
{
    typedef LoggerDecoratorBase<TNextLayer> Base;
public:

    /// @brief Constructor
    /// @details Forwards all the parameters to the constructor of the next layer
    /// @param[in] params Zero or more parameters to be forwarded to the next layer.
    /// @note Exception guarantee: Strong
    template<typename... TParams>
    LevelStringPrefixer(TParams&&... params);

    /// @brief Prints log level in square brackets prior
    ///        to redirecting the request to the next layer.
    /// @param[in] level requested logging level
    void begin(Level level);
};

/// @}

// Implementation
template <typename TNextLayer>
template<typename... TParams>
LevelStringPrefixer<TNextLayer>::LevelStringPrefixer(
    TParams&&... params)
    : Base(std::forward<TParams>(params)...)
{
}

template <typename TNextLayer>
void LevelStringPrefixer<TNextLayer>::begin(
    Level level)
{
    static const char* const Strings[NumOfLogLevels] = {
        "[TRACE] ",
        "[DEBUG] ",
        "[INFO] ",
        "[WARNING] ",
        "[ERROR] "
    };

    if ((level < NumOfLogLevels) && (Strings[level] != nullptr)) {
        Base::stream() << Strings[level];
    }

    Base::begin(level);
}

}  // namespace log

}  // namespace util

}  // namespace embxx
