//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

#pragma once

/// @file util/Assert.h
/// This file contains classes required for generic custom assertion
/// functionality

#include <cassert>
#include <type_traits>
#include <utility>

namespace embxx
{

namespace util
{

/// @addtogroup util
/// @{

/// @brief Base class for any custom assertion behaviour.
/// @details In order to implement custom assertion failure behaviour it
///          is necessary to inherit from this class and override
///          fail() virtual member function.
/// @headerfile embxx/util/Assert.h
class Assert
{
public:
    /// @brief Destructor
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw.
    virtual ~Assert() {}

    /// @brief Pure virtual function to be called when assertion fails.
    /// @param[in] expr Assertion condition/expression
    /// @param[in] file File name
    /// @param[in] line Line number of the assert statement.
    /// @param[in] function Function name.
    /// @note Thread safety: Safe.
    /// @note Exception guarantee: No throw.
    virtual void fail(
        const char* expr,
        const char* file,
        unsigned int line,
        const char* function) = 0;

private:
};

/// @cond DOCUMENT_ASSERT_MANAGER
template <class T>
class BasicAssertManager
{
public:

    static BasicAssertManager& instance();

    BasicAssertManager(const BasicAssertManager& other) = delete;
    BasicAssertManager& operator=(const BasicAssertManager& other) = delete;

    Assert* reset(Assert* newAssert = nullptr);

    Assert* getAssert();

    bool hasAssertRegistered() const;

    static inline
    void infiniteLoop()
    {
        while (true) {};
    }

private:
    BasicAssertManager();

    Assert* assert_;
};

typedef BasicAssertManager<void> AssertManager;

/// @endcond

/// @brief Enable new assertion behaviour.
/// @details Instantiate object of this class to enable new behaviour of
///          assertion failure.
/// @tparam TAssert Class derived from Assert that implements new custom
///                 behaviour of the assertion failure.
/// @pre TAssert class must be derived from util::Assert.
/// @note Thread safety: Unsafe. If there are multiple custom assertion failures
///       defined in single binary, all of them must be done in the main thread
///       and preferable before any other threads are created.
/// @headerfile embxx/util/Assert.h
template < typename TAssert>
class EnableAssert
{
    static_assert(std::is_base_of<Assert, TAssert>::value,
        "TAssert class must be derived class of Assert");
public:
    /// Type of assert object.
    typedef TAssert AssertType;

    /// @brief Constructor
    /// @details Registers new assertion failure behaviour. It forwards
    ///          all the provided parameters to the constructor of embedded
    ///          assertion object of type TAssert.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Depends on exception guarantee of the
    ///       TAssert's constructor.
    template<typename ...Params>
    EnableAssert(Params... args);

    /// @brief Destructor
    /// @details Restores the assertion behaviour that was recorded during
    ///          the instantiation of this object.
    /// @note Thread safety: Unsafe
    /// @note Exception guarantee: Depends on exception guarantee of the
    ///       TAssert's destructor.
    ~EnableAssert();


    /// @brief Provides reference to internal Assert object
    /// @return Reference to object of type TAssert.
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw.
    AssertType& getAssert();

private:
    AssertType assert_;
    Assert* prevAssert_;
};


#ifndef NDEBUG

/// @cond DOCUCMENT_AM_ASSERT_FUNCTION
#ifndef __ASSERT_FUNCTION
#define GASSERT_FUNCTION_STR __FUNCTION__
#else // #ifndef __ASSERT_FUNCTION
#define GASSERT_FUNCTION_STR __ASSERT_FUNCTION
#endif // #ifndef __ASSERT_FUNCTION

#ifndef NOSTDLIB
#define GASSERT_FAIL_FUNC(expr) assert(expr)
#else // #ifndef NOSTDLIB
#define GASSERT_FAIL_FUNC(expr) embxx::util::AssertManager::instance().infiniteLoop()
#endif // #ifndef NOSTDLIB

/// @endcond

/// @brief Generic assert macro
/// @details Will use custom assertion failure behaviour if such is defined,
///          otherwise it will use standard "assert()" macro.
///          In case NOSTDLIB is defined and no custom assertion failure was
///          enabled, infinite loop will be executed.
/// @param expr Boolean expression
#define GASSERT(expr) \
    ((expr)                               \
      ? static_cast<void>(0)                     \
      : (embxx::util::AssertManager::instance().hasAssertRegistered() \
            ? embxx::util::AssertManager::instance().getAssert()->fail( \
                #expr, __FILE__, __LINE__, GASSERT_FUNCTION_STR) \
            : GASSERT_FAIL_FUNC(expr)))

#else // #ifndef NDEBUG

#define GASSERT(expr) static_cast<void>(0)

#endif // #ifndef NDEBUG

/// @}

// Implementation

/// @cond DOCUMENT_ASSERT_MANAGER
template <typename T>
BasicAssertManager<T>& BasicAssertManager<T>::instance()
{
    static AssertManager mgr;
    return mgr;
}

template <typename T>
Assert* BasicAssertManager<T>::reset(Assert* newAssert)
{
    Assert* old = assert_;
    assert_ = newAssert;
    return old;
}

template <typename T>
Assert* BasicAssertManager<T>::getAssert()
{
    return assert_;
}

template <typename T>
bool BasicAssertManager<T>::hasAssertRegistered() const
{
    return (assert_ != nullptr);
}

template <typename T>
BasicAssertManager<T>::BasicAssertManager()
    : assert_(nullptr)
{
}



/// @endcond

template <typename TAssert>
template<typename ...Params>
EnableAssert<TAssert>::EnableAssert(Params... args)
    : assert_(std::forward<Params>(args)...),
      prevAssert_(AssertManager::instance().reset(&assert_))
{
}

template < typename TAssert>
EnableAssert<TAssert>::~EnableAssert()
{
    AssertManager::instance().reset(prevAssert_);
}

template < typename TAssert>
typename EnableAssert<TAssert>::AssertType&
EnableAssert<TAssert>::getAssert()
{
    return assert_;
}


}  // namespace util

}  // namespace embxx
