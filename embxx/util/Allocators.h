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

/// @file embxx/util/Allocators.h
/// This file contains definition of various allocators.

#pragma once

#include <new>
#include <cstddef>
#include <memory>
#include <type_traits>
#include <tuple>

#include "embxx/util/Assert.h"
#include "embxx/util/AlignedUnion.h"
#include "embxx/util/Tuple.h"

namespace embxx
{

namespace util
{

/// @addtogroup util
/// @{

/// @brief Object allocation policy that uses dynamic memory allocation.
/// @details The newly allocated object is returned wrapped in
///          standard std::unique_ptr with default deleter.
/// @headerfile embxx/util/Allocators.h
class DynMemAllocator
{
public:

    /// @brief Allocation function
    /// @details Uses new operator to dynamically allocate and initialise object
    /// @tparam TObj Type of object to by allocated
    /// @tparam TArgs Types of the parameters required to create an object.
    /// @return std::unique_ptr to the allocated object.
    template <typename TObj, typename... TArgs>
    std::unique_ptr<TObj> alloc(TArgs&&... args)
    {
        return std::unique_ptr<TObj>(new TObj(std::forward<TArgs>(args)...));
    }
};

/// @brief Object allocation policy that uses "in place" object construction.
/// @details The newly created object is returned wrapped in
///          standard std::unique_ptr with a deleter that calls destructor of
///          the allocated object. This allocator is unsafe, it expects pointer
///          to the allocation space to be provided and doesn't check
///          alignment or size errors for the allocated objects.
/// @headerfile embxx/util/Allocators.h
class BasicInPlaceAllocator
{
public:

    /// @cond DOCUMENT_ASSERT_MANAGER

    /// @brief Deleter class
    template <typename T>
    class Deleter
    {
        template<typename U>
        friend class Deleter;

    public:
        /// Constructor used by BasicInPlaceAllocator to create std::unique_ptr
        Deleter(bool* allocated = nullptr)
            : allocated_(allocated)
        {
        }

        /// Copy constructor is deleted
        Deleter(const Deleter& other) = delete;

        template <typename U>
        Deleter(Deleter<U>&& other)
            : allocated_(other.allocated_)
        {
            static_assert(std::is_base_of<T, U>::value ||
                          std::is_base_of<U, T>::value ||
                          std::is_convertible<U, T>::value ||
                          std::is_convertible<T, U>::value ,
                "To make Deleter convertible, their template parameters "
                "must be convertible.");

            other.allocated_ = nullptr;
        }

        ~Deleter()
        {
            GASSERT(allocated_ == nullptr);
        }

        /// Copy assignment is deleted
        Deleter& operator=(const Deleter& other) = delete;

        template <typename U>
        Deleter& operator=(Deleter<U>&& other)
        {
            static_assert(std::is_base_of<T, U>::value ||
                          std::is_base_of<U, T>::value ||
                          std::is_convertible<U, T>::value ||
                          std::is_convertible<T, U>::value ,
                "To make Deleter convertible, their template parameters "
                "must be convertible.");

            if (reinterpret_cast<void*>(this) == reinterpret_cast<const void*>(&other)) {
                return *this;
            }

            GASSERT(allocated_ == nullptr);
            allocated_ = other.allocated_;
            other.allocated_ = nullptr;
            return *this;
        }

        /// @brief Deletion operator
        /// @details Executes destructor of the deleted object
        void operator()(T* obj) {
            GASSERT(allocated_ != nullptr);
            GASSERT(*allocated_);
            obj->~T();
            *allocated_ = false;
            allocated_ = nullptr;
        }

    private:
        bool* allocated_;
    };
    /// @endcond


    /// @brief Constructor
    /// @param place Pointer to a space where allocation should happen. If
    ///              0 (default) every allocation will fail until
    ///              setAllocPlace() with valid pointer is called.
    BasicInPlaceAllocator(void* place = 0)
        : place_(place),
          allocated_(false)
    {
    }

    /// Destructor
    ~BasicInPlaceAllocator()
    {
        GASSERT(!allocated_);
    }

    /// @brief Set pointer to allocation place
    void setAllocPlace(void* place = 0)
    {
        place_ = place;
    }

    /// @brief Allocation function
    /// @details Uses in place object construction
    /// @tparam TObj Type of object to by constructed
    /// @tparam TArgs Types of the parameters required to create an object.
    /// @return std::unique_ptr to constructed object with custom deleter that
    ///         calls the destructor of the object.
    /// @pre TObj must fit into the allocation space, pointer to which
    ///      was provided in the construction of the allocator.
    /// @pre Allocation place, pointer to which was provided in the
    ///      construction of the allocator, must have at least the same
    ///      alignment as required by TObj class.
    template <typename TObj, typename... TArgs>
    std::unique_ptr<TObj, Deleter<TObj> > alloc(TArgs&&... args)
    {
        typedef Deleter<TObj> Del;
        std::unique_ptr<TObj, Del> ptr(nullptr, Del());
        if ((place_ != nullptr) && (!allocated_)) {
            ptr.reset(new (place_) TObj(std::forward<TArgs>(args)...));
            allocated_ = true;
            ptr.get_deleter() = Del(&allocated_);
        }
        return std::move(ptr);
    }

private:
    void* place_;
    bool allocated_;
};

/// @brief Object allocation policy that uses "in place" object construction.
/// @details Much safer "in place" allocator than BasicInPlaceAllocator. It
///          allocates required space with required alignment as private
///          data and uses BasicInPlaceAllocator to create objects in
///          the allocated space.
/// @tparam TSize Required size.
/// @tparam TAlignment Required alignment. By default the alignment will
///         be the same as alignment of "double", usually 8 bytes.
/// @headerfile embxx/util/Allocators.h
template <std::size_t TSize,
          std::size_t TAlignment = std::alignment_of<double>::value>
class InPlaceAllocator
{
public:

    /// Using BasicInPlaceAllocator
    typedef BasicInPlaceAllocator Allocator;

    /// Constructor
    InPlaceAllocator()
        : allocator_(&place_)
    {
    }

    /// @brief Allocation function
    /// @details Uses in place object construction
    /// @tparam TObj Type of object to by constructed
    /// @tparam TArgs Types of the parameters required to create an object.
    /// @return std::unique_ptr to constructed object with custom deleter that
    ///         calls the destructor of the object.
    /// @pre sizeof(TObj) <= TSize
    /// @pre std::alignment_of<TObj>::value <= TAlignment.
    template <typename TObj, typename... TArgs>
    auto alloc(TArgs&&... args) -> decltype(BasicInPlaceAllocator().alloc<TObj>(std::forward<TArgs>(args)...))
    {
        static_assert(sizeof(TObj) <= sizeof(place_),
                                "Must be enough space for allocation");

        static_assert(std::alignment_of<TObj>::value <= TAlignment,
                                "Failed alignment requirements");


        return allocator_.alloc<TObj>(std::forward<TArgs>(args)...);
    }

private:
    typename std::aligned_storage<TSize, TAlignment>::type place_;
    BasicInPlaceAllocator allocator_;
};

/// @brief Object allocation policy that uses "in place" object construction.
/// @details It receives all the types it can allocate wrapped in std::tuple
///          in single template parameter, calculates the required size and
///          alignment to be able to safely allocated any of the required types
///          and uses InPlaceAllocator for the allocations.
/// @tparam TTuple std::tuple<...> with all the types this allocator can
///         allocate
/// @headerfile embxx/util/Allocators.h
template <typename TTuple>
class SpecificInPlaceAllocator
{
    static_assert(IsTuple<TTuple>::Value, "TTuple must be std::tuple");
    typedef typename TupleAsAlignedUnion<TTuple>::Type AlignedStorage;

public:

    /// Using InPlaceAllocator
    typedef InPlaceAllocator<sizeof(AlignedStorage), std::alignment_of<AlignedStorage>::value> Allocator;

    /// @brief Allocation function
    /// @details Uses in place object construction
    /// @tparam TObj Type of object to by constructed
    /// @tparam TArgs Types of the parameters required to create an object.
    /// @return std::unique_ptr to constructed object with custom deleter that
    ///         calls the destructor of the object.
    /// @pre TObj was included in TTuple.
    template <typename TObj, typename... TArgs>
    auto alloc(TArgs&&... args) -> decltype(Allocator().template alloc<TObj>(std::forward<TArgs>(args)...))
    {
        static_assert(IsInTuple<TObj, TTuple>::Value,
                    "TObj must be included in TTuple");

        return allocator_.alloc<TObj>(std::forward<TArgs>(args)...);
    }

private:
    Allocator allocator_;
};

/// @}

}  // namespace util

}  // namespace embxx
