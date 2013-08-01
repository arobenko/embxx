//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file util/assert/CxxTestAssert.h
/// Contains definition of CxxTestAssert.

#pragma once

#include <sstream>
#include "embxx/util/Assert.h"
#include "cxxtest/TestSuite.h"

namespace embxx
{

namespace util
{

namespace assert
{

/// @addtogroup util
///
/// @{


/// @brief Custom assert class to be used with CxxTest unittesting framework.
/// @details The main purpose of this class is to override the default
///          assertion failure behaviour with call to TS_FAIL provided by
///          CxxTest framework. Please don't use this class directly, use
///          util::assert::CxxTestAssert typedef.
/// @tparam T Dummy template parameter. It's main purpose to ensure that
///         the code of this class is not generated if not used.
/// @headerfile embxx/util/assert/CxxTestAssert.h
template <class T>
class BasicCxxTestAssert : public util::Assert
{
public:
    virtual ~BasicCxxTestAssert() {}

    /// @brief Implementation of the fail functionality.
    /// @details Calls TS_FAIL() with assert message similar to one produced
    ///          by standard assert().
    virtual void fail(
        const char* expr,
        const char* file,
        unsigned int line,
        const char* function) override;
};

/// Definition of CxxTestAssert class.
/// @related BasicCxxTestAssert
typedef BasicCxxTestAssert<void> CxxTestAssert;

/// @}

// Implementation

template <class T>
void BasicCxxTestAssert<T>::fail(
    const char* expr,
    const char* file,
    unsigned int line,
    const char* function)
{
    std::stringstream stream;
    stream << file << ":" << line <<
           " " << function << ": " <<
           "Assertion \'" << expr << "\' " <<
           "failed.";
    TS_FAIL(stream.str().c_str());
}

}  // namespace assert

}  // namespace util

}  // namespace embxx
