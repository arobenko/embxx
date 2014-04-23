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

/// @file embxx/util/StaticFunction.h
/// Provides StaticFunction class.

#pragma once

#include <type_traits>
#include <new>
#include <functional>

#include "embxx/util/Assert.h"

namespace embxx
{

namespace util
{

/// @addtogroup util
/// @{

/// @brief Generic declaration of StaticFunction.
/// @details This declaration doesn't have a body, see specialisation.
/// @headerfile embxx/util/StaticFunction.h
template <typename TSignature, std::size_t TSize = sizeof(void*) * 3>
class StaticFunction;

/// @brief Static Function.
/// @details StaticFunction is similar to std::function. However it doesn't
///          use dynamic memory allocation, hence it must receive amount of space
///          required to store provided functor object.
///
///          This is template specialisation of the following class definition
///          @code
///          // TSignature is a combination of return value an arguments: TRet(TArgs...)
///          template <typename TSignature, std::size_t TSize = 16>
///          class StaticFunction;
///          @endcode
/// @tparam TSize Size of the space required to store provided functor.
/// @tparam TRet Return type of the function
/// @tparam TArgs Argument types
/// @headerfile embxx/util/StaticFunction.h
template <std::size_t TSize, typename TRet, typename... TArgs>
class StaticFunction<TRet (TArgs...), TSize>
{
public:
    /// @brief Result type
    typedef TRet result_type;

    static const std::size_t Size = TSize;

    /// @brief Default constructor
    StaticFunction();

    /// @brief Constructs StaticFunction object out of provided functor
    /// @pre TFunc invocation must have the same signature as StaticFunction
    /// @pre @code sizeof(TFunc) <= TSize @endcode
    template <typename TFunc>
    explicit StaticFunction(TFunc&& func);

    /// @brief Copy constructor
    StaticFunction(const StaticFunction& other);

    /// @brief Move constructor
    StaticFunction(StaticFunction&& other);

    /// @brief Destructor
    ~StaticFunction();

    /// @brief Copy assignment operator
    StaticFunction& operator=(const StaticFunction& other);

    /// @brief Non-const param copy assignment operator
    StaticFunction& operator=(StaticFunction& other);

    /// @brief Move assignment operator
    /// @post Other function becomes invalid: @code (!other) == true @endcode
    StaticFunction& operator=(StaticFunction&& other);

    /// @brief Invalidates current function.
    /// @post This function becomes invalid: @code (!(*this)) == true @endcode
    StaticFunction& operator=(std::nullptr_t);

    /// @brief Assigns new functor to current function using move semantics.
    /// @pre TFunc invocation must have the same signature as StaticFunction
    /// @pre @code sizeof(TFunc) <= TSize @endcode
    /// @post This function becomes valid: @code (!(*this)) == false @endcode
    template <typename TFunc>
    StaticFunction& operator=(TFunc&& func);

    /// @brief Assigns new functor to current function using copy semantics.
    /// @pre TFunc invocation must have the same signature as StaticFunction
    /// @pre @code sizeof(TFunc) <= TSize @endcode
    /// @post This function becomes valid: @code (!(*this)) == false @endcode
    template <typename TFunc>
    StaticFunction& operator=(std::reference_wrapper<TFunc> func);

    /// @brief Boolean conversion operator.
    /// @return Returns true if and only if current function is valid, i.e.
    ///         may be invoked using operator().
    operator bool() const;

    /// @brief Negation operator.
    /// @return Returns true if and only if current function is invalid, i.e.
    ///         may NOT be invoked using operator().
    bool operator!() const;

    /// @brief Function invocation operator.
    /// @details Invokes operator() of the stored functor with provided arguments
    /// @return What functor returns
    /// @pre The function object is valid, i.e. has functor assigned to it.
    TRet operator()(TArgs... args) const;

    /// @brief Non-const version of operator().
    TRet operator()(TArgs... args);

private:

    /// @cond DOCUMENT_STATIC_FUNCTION_INVOKER
    class Invoker
    {
    public:
        virtual ~Invoker();
        virtual TRet exec(TArgs... args) const = 0;
        virtual TRet exec(TArgs... args) = 0;
        virtual void copyTo(void* other) const = 0;
        virtual void moveTo(void* other) = 0;
    private:
    };

    template <typename TBound>
    class InvokerBound : public Invoker
    {
    public:
        template <typename TFunc>
        InvokerBound(TFunc&& func);
        InvokerBound(const InvokerBound&) = default;
        InvokerBound(InvokerBound&&) = default;
        virtual ~InvokerBound();
        virtual TRet exec(TArgs... args) const;
        virtual TRet exec(TArgs... args);
        virtual void copyTo(void* other) const;
        virtual void moveTo(void* other);
    private:
        TBound func_;
    };
    /// @endcond

    static const std::size_t StorageAreaSize = TSize + sizeof(Invoker);
    typedef typename
        std::aligned_storage<
            StorageAreaSize,
            std::alignment_of<Invoker>::value
        >::type StorageType;

    Invoker* getInvoker();
    const Invoker* getInvoker() const;
    void destroyHandler();
    template <typename TFunc>
    void assignHandler(TFunc&& func);

    StorageType handler_;
    bool valid_;
};

/// @}

// Implementation
template <std::size_t TSize, typename TRet, typename... TArgs>
StaticFunction<TRet (TArgs...), TSize>::StaticFunction()
    : valid_(false)
{
}

template <std::size_t TSize, typename TRet, typename... TArgs>
template <typename TFunc>
StaticFunction<TRet (TArgs...), TSize>::StaticFunction(TFunc&& func)
    : valid_(true)
{
    assignHandler(std::forward<TFunc>(func));
}

template <std::size_t TSize, typename TRet, typename... TArgs>
StaticFunction<TRet (TArgs...), TSize>::StaticFunction(
    const StaticFunction& other)
    : valid_(other.valid_)
{
    if (valid_) {
        auto otherInvoker = other.getInvoker();
        otherInvoker->copyTo(&handler_);
    }
}

template <std::size_t TSize, typename TRet, typename... TArgs>
StaticFunction<TRet (TArgs...), TSize>::StaticFunction(
    StaticFunction&& other)
    : valid_(other.valid_)
{
    if (valid_) {
        auto otherInvoker = other.getInvoker();
        otherInvoker->moveTo(&handler_);
        other = nullptr;
    }
}

template <std::size_t TSize, typename TRet, typename... TArgs>
StaticFunction<TRet (TArgs...), TSize>::~StaticFunction()
{
    destroyHandler();
}

template <std::size_t TSize, typename TRet, typename... TArgs>
StaticFunction<TRet (TArgs...), TSize>&
StaticFunction<TRet (TArgs...), TSize>::operator=(const StaticFunction& other)
{
    if (&other == this) {
        return *this;
    }

    destroyHandler();
    valid_ = other.valid_;
    if (valid_) {
        auto otherInvoker = other.getInvoker();
        GASSERT(otherInvoker != nullptr);
        otherInvoker->copyTo(&handler_);
    }
    return *this;
}

template <std::size_t TSize, typename TRet, typename... TArgs>
StaticFunction<TRet (TArgs...), TSize>&
StaticFunction<TRet (TArgs...), TSize>::operator=(StaticFunction& other)
{
    return operator=(static_cast<const StaticFunction&>(other));
}

template <std::size_t TSize, typename TRet, typename... TArgs>
StaticFunction<TRet (TArgs...), TSize>&
StaticFunction<TRet (TArgs...), TSize>::operator=(StaticFunction&& other)
{
    if (&other == this) {
        return *this;
    }

    destroyHandler();
    valid_ = other.valid_;
    if (valid_) {
        auto otherInvoker = other.getInvoker();
        GASSERT(otherInvoker != nullptr);
        otherInvoker->moveTo(&handler_);
        other = nullptr;
    }
    return *this;
}

template <std::size_t TSize, typename TRet, typename... TArgs>
StaticFunction<TRet (TArgs...), TSize>&
StaticFunction<TRet (TArgs...), TSize>::operator=(std::nullptr_t)
{
    destroyHandler();
    valid_ = false;
    return *this;
}

template <std::size_t TSize, typename TRet, typename... TArgs>
template <typename TFunc>
StaticFunction<TRet (TArgs...), TSize>&
StaticFunction<TRet (TArgs...), TSize>::operator=(TFunc&& func)
{
    destroyHandler();
    assignHandler(std::forward<TFunc>(func));
    valid_ = true;
    return *this;
}

template <std::size_t TSize, typename TRet, typename... TArgs>
template <typename TFunc>
StaticFunction<TRet (TArgs...), TSize>&
StaticFunction<TRet (TArgs...), TSize>::operator=(
    std::reference_wrapper<TFunc> func)
{
    destroyHandler();
    assignHandler(std::forward<TFunc>(func));
    valid_ = true;
    return *this;
}

template <std::size_t TSize, typename TRet, typename... TArgs>
StaticFunction<TRet (TArgs...), TSize>::operator bool() const
{
    return valid_;
}

template <std::size_t TSize, typename TRet, typename... TArgs>
bool StaticFunction<TRet (TArgs...), TSize>::operator!() const
{
    return !valid_;
}

template <std::size_t TSize, typename TRet, typename... TArgs>
TRet StaticFunction<TRet (TArgs...), TSize>::operator()(
    TArgs... args) const
{
    GASSERT(valid_);
    auto invoker = getInvoker();
    return invoker->exec(std::forward<TArgs>(args)...);
}

template <std::size_t TSize, typename TRet, typename... TArgs>
TRet StaticFunction<TRet (TArgs...), TSize>::operator()(
    TArgs... args)
{
    GASSERT(valid_);
    auto invoker = getInvoker();
    return invoker->exec(std::forward<TArgs>(args)...);
}

/// @cond DOCUMENT_STATIC_FUNCTION_INVOKER
template <std::size_t TSize, typename TRet, typename... TArgs>
StaticFunction<TRet (TArgs...), TSize>::Invoker::~Invoker()
{
}

template <std::size_t TSize, typename TRet, typename... TArgs>
template <typename TBound>
template <typename TFunc>
StaticFunction<TRet (TArgs...), TSize>::InvokerBound<TBound>::InvokerBound(
    TFunc&& func)
    : func_(std::forward<TFunc>(func))
{
}

template <std::size_t TSize, typename TRet, typename... TArgs>
template <typename TBound>
StaticFunction<TRet (TArgs...), TSize>::InvokerBound<TBound>::~InvokerBound()
{
}

template <std::size_t TSize, typename TRet, typename... TArgs>
template <typename TBound>
TRet StaticFunction<TRet (TArgs...), TSize>::InvokerBound<TBound>::exec(
    TArgs... args) const
{
    return func_(std::forward<TArgs>(args)...);
}

template <std::size_t TSize, typename TRet, typename... TArgs>
template <typename TBound>
TRet StaticFunction<TRet (TArgs...), TSize>::InvokerBound<TBound>::exec(
    TArgs... args)
{
    return func_(std::forward<TArgs>(args)...);
}

template <std::size_t TSize, typename TRet, typename... TArgs>
template <typename TBound>
void StaticFunction<TRet (TArgs...), TSize>::InvokerBound<TBound>::copyTo(
    void* place) const
{
    auto otherInvoker = new (place) InvokerBound(*this);
    static_cast<void>(otherInvoker);
}

template <std::size_t TSize, typename TRet, typename... TArgs>
template <typename TBound>
void StaticFunction<TRet (TArgs...), TSize>::InvokerBound<TBound>::moveTo(
    void* place)
{
    auto otherInvoker = new (place) InvokerBound(std::move(*this));
    static_cast<void>(otherInvoker);
}
/// @endcond

template <std::size_t TSize, typename TRet, typename... TArgs>
typename StaticFunction<TRet (TArgs...), TSize>::Invoker*
StaticFunction<TRet (TArgs...), TSize>::getInvoker()
{
    return reinterpret_cast<Invoker*>(&handler_);
}

template <std::size_t TSize, typename TRet, typename... TArgs>
const typename StaticFunction<TRet (TArgs...), TSize>::Invoker*
StaticFunction<TRet (TArgs...), TSize>::getInvoker() const
{
    return reinterpret_cast<const Invoker*>(&handler_);
}

template <std::size_t TSize, typename TRet, typename... TArgs>
void StaticFunction<TRet (TArgs...), TSize>::destroyHandler()
{
    if (valid_) {
        auto invoker = getInvoker();
        invoker->~Invoker();
    }
}

template <std::size_t TSize, typename TRet, typename... TArgs>
template <typename TFunc>
void StaticFunction<TRet (TArgs...), TSize>::assignHandler(TFunc&& func)
{
    typedef StaticFunction<TRet (TArgs...), TSize> ThisType;
    typedef typename std::decay<TFunc>::type DecayedFuncType;
    typedef InvokerBound<DecayedFuncType> InvokerBoundType;

    static_assert(!std::is_same<ThisType, DecayedFuncType>::value,
        "Wrong function invocation");

    static_assert(sizeof(InvokerBoundType) <= StorageAreaSize,
        "Increase the TSize template argument of the StaticFucntion");

    static_assert(alignof(Invoker) == alignof(InvokerBoundType),
        "Alignment requirement for Invoker object must be the same as "
        "alignment requirement for InvokerBoundType type object");

    auto handlerPtr = new (&handler_) InvokerBoundType(std::forward<TFunc>(func));
    static_cast<void>(handlerPtr);

}

}  // namespace util

}  // namespace embxx



