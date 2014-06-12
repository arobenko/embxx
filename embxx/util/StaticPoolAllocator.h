//
// Copyright 2014 (C). Alex Robenko. All rights reserved.
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

#include <cstddef>
#include <type_traits>
#include <array>
#include <bitset>
#include <limits>

#include "embxx/util/Assert.h"

namespace embxx
{

namespace util
{

namespace details
{

template <typename TTag, typename T, std::size_t TSize>
struct StaticPoolAllocatorStorage
{
    typedef typename
        std::aligned_storage<
            sizeof(T),
            std::alignment_of<T>::value
        >::type CellType;

    static std::array<CellType, TSize> items_;
    static std::bitset<TSize> allocFlags_;
};

}  // namespace details

template <typename TTag, typename T = void, std::size_t TSize = 1>
class StaticPoolAllocator
{
    static_assert(
        std::is_same<typename std::remove_reference<T>::type, T>::value,
        "Template parameter T to embxx::util::ProvidedSpaceStaticPoolAllocator "
        "mustn't be reference");

    static_assert(
        std::is_same<typename std::remove_pointer<T>::type, T>::value,
        "Template parameter T to embxx::util::ProvidedSpaceStaticPoolAllocator "
        "mustn't be pointer");

    typedef details::StaticPoolAllocatorStorage<TTag, T, TSize> Storage;

public:
    typedef T value_type;
    typedef T* pointer;
    typedef const T* const_pointer;
    typedef T& reference;
    typedef const T& const_reference;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;

    template <typename U>
    struct rebind
    {
        typedef StaticPoolAllocator<TTag, U, TSize> other;
    };

    StaticPoolAllocator() = default;
    StaticPoolAllocator(const StaticPoolAllocator&) = default;
    StaticPoolAllocator(StaticPoolAllocator&&) = default;
    ~StaticPoolAllocator() = default;
    StaticPoolAllocator& operator=(const StaticPoolAllocator&) = default;
    StaticPoolAllocator& operator=(StaticPoolAllocator&&) = default;

    pointer allocate(size_type num)
    {
        if (max_size() < num) {
            return nullptr;
        }

        auto allocBitset = numToBitset(num);

        std::size_t idx = 0;
        while (idx <= (Storage::allocFlags_.size() - num)) {
            auto checkBitmask = Storage::allocFlags_ & allocBitset;
            if (checkBitmask.none()) {
                Storage::allocFlags_ |= allocBitset;
                return &Storage::items_[idx];
            }

            allocBitset <<= 1U;
            ++idx;
        }
        return nullptr;
    }

    void deallocate(pointer ptr, size_type num)
    {
        auto releaseBitset = numToBitset(num);

        auto& items = Storage::items_;
        auto idxTmp = std::distance(&items[0], ptr);
        GASSERT((0 <= idxTmp) && ((idxTmp + num) < items.size()));
        auto idx = static_cast<size_type>(idxTmp);
        releaseBitset <<= idx;

        GASSERT((Storage::allocFlags_ & releaseBitset) == releaseBitset);
        Storage::allocFlags_ ^= releaseBitset;
    }

    constexpr size_type max_size() const
    {
        return TSize;
    }

    template< class U, class... Args >
    void construct( U* p, Args&&... args )
    {
        ::new((void *)p) U(std::forward<Args>(args)...);
    }

    template< class U>
    void destroy(U* p)
    {
        p->~U();
    }

private:
    decltype(Storage::allocFlags_) numToBitset(size_type num)
    {
        typedef unsigned long long BitsetParamType;

        static const std::size_t ChunkSize =
            std::numeric_limits<BitsetParamType>::digits;

        typedef decltype(Storage::allocFlags_) Bitset;
        Bitset bitset;
        auto remNum = num;
        while (remNum >= ChunkSize) {
            static const auto mask = ~(static_cast<BitsetParamType>(0));
            bitset <<= ChunkSize;
            bitset |= Bitset(mask);
            remNum -= ChunkSize;
        }

        auto lastMask = (static_cast<BitsetParamType>(1U) << remNum) - 1;
        bitset <<= remNum;
        bitset |= Bitset(lastMask);
        return bitset;
    }
};

template <typename TTag, std::size_t TSize>
class StaticPoolAllocator<TTag, void, TSize>
{
public:
    typedef void value_type;
    typedef void* pointer;
    typedef const void* const_pointer;

    template <typename U>
    struct rebind
    {
        typedef StaticPoolAllocator<TTag, U, TSize> other;
    };

    StaticPoolAllocator() = default;
    StaticPoolAllocator(const StaticPoolAllocator&) = default;
    StaticPoolAllocator(StaticPoolAllocator&&) = default;
    ~StaticPoolAllocator() = default;
    StaticPoolAllocator& operator=(const StaticPoolAllocator&) = default;
    StaticPoolAllocator& operator=(StaticPoolAllocator&&) = default;
};

template <typename TTag, typename T, std::size_t TSize>
bool operator==(
    const StaticPoolAllocator<TTag, T, TSize>,
    const StaticPoolAllocator<TTag, T, TSize>)
{
    return true;
}

template <typename TTag1, typename T1, std::size_t TSize1, typename TTag2, typename T2, std::size_t TSize2>
bool operator==(
    const StaticPoolAllocator<TTag1, T1, TSize1>,
    const StaticPoolAllocator<TTag2, T2, TSize2>)
{
    return false;
}

template <typename TTag1, typename T1, std::size_t TSize1, typename TTag2, typename T2, std::size_t TSize2>
bool operator!=(
    const StaticPoolAllocator<TTag1, T1, TSize1> a1,
    const StaticPoolAllocator<TTag2, T2, TSize2> a2)
{
    return !(a1 == a2);
}

}  // namespace util

}  // namespace embxx


