//
// Copyright 2012 (C). Alex Robenko. All rights reserved.
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

/// @file embxx/container/StaticQueueStlAdapter.h
/// STL Adapter a wrapper to StaticQueue that adapts an existing
/// interface to the one used by STL containers.

#pragma once

#include <iostream>

namespace embxx
{

namespace container
{

/// @addtogroup container
/// @{

/// @brief StaticQueue container adapter that provides STL interface.
/// @details The adapter just forwards all the requests to appropriate
///          function in the adapter. All its functions are declared to be
///          inline.
/// @tparam TContainer Container type
template <typename TContainer>
class StaticQueueStlAdapter
{
public:
    // Types

    /// @brief Container type
    typedef TContainer container_type;

    /// @brief Value type
    typedef typename TContainer::ValueType value_type;

    /// @brief Size type
    typedef typename TContainer::SizeType size_type;

    /// @brief Reference type
    typedef typename TContainer::Reference reference;

    /// @brief Const reference type
    typedef typename TContainer::ConstReference const_reference;

    /// @brief Pointer type
    typedef typename TContainer::Pointer pointer;

    /// @brief Const pointer type
    typedef typename TContainer::ConstPointer const_pointer;

    /// @brief Iterator
    typedef typename TContainer::LinearisedIterator iterator;

    /// @brief Const iterator
    typedef typename TContainer::ConstLinearisedIterator const_iterator;

    /// @brief Reverse iterator
    typedef typename TContainer::ReverseLinearisedIterator reverse_iterator;

    /// @brief Const reverse iterator
    typedef typename TContainer::ConstReverseLinearisedIterator const_reverse_iterator;

    // Member functions

    /// @brief Constructor
    /// @details Forwards all the arguments to the underlying container.
    ///          It also serves as copy/move constructor for the wrapper
    ///          thanks to the "operator TContainer&()".
    /// @param args all the arguments.
    template <typename... TParams>
    inline
    StaticQueueStlAdapter(TParams&&... args);

    /// @brief Conversion operator
    /// @details Returns reference to underlying container.
    /// @return Reference to underlying container.
    /// @note Thread safety: Safe
    /// @note Exception guarantee: No throw
    inline operator TContainer&();

    /// @brief Copy assignment operator
    /// @details Forwards the assignment request to underlying container.
    /// @param other Other container wrapped in StaticQueueStlAdapter.
    inline StaticQueueStlAdapter& operator=(const StaticQueueStlAdapter& other);

    /// @brief Move assignment operator
    /// @details Forwards the assignment request to underlying container.
    /// @param other Other container wrapped in StaticQueueStlAdapter.
    inline StaticQueueStlAdapter& operator=(StaticQueueStlAdapter&& other);

    /// @brief Copy assignment operator
    /// @details Forwards the assignment request to underlying container.
    /// @param other Other container NOT wrapped in StaticQueueStlAdapter.
    inline StaticQueueStlAdapter& operator=(const TContainer& other);

    /// @brief Move assignment operator
    /// @details Forwards the assignment request to underlying container.
    /// @param other Other container NOT wrapped in StaticQueueStlAdapter.
    inline StaticQueueStlAdapter& operator=(TContainer&& other);

    /// @brief Retrieve underlying container.
    /// @return Reference to underlying container.
    /// @note Thread safety: Safe.
    /// @note Exception guarantee: No throw.
    inline TContainer& container();

    /// @brief Const version of container().
    inline const TContainer& container() const;

    /// @brief Push at the back function
    /// @details Forwards request to pushBack() function of the
    ///          underlying container.
    /// @param args All the parameters. Can contain both l-values and r-values.
    template <typename... TParams>
    inline void push_back(TParams&&... args);

    /// @brief Push at the front function
    /// @details Forwards request to pushFront() function of the
    ///          underlying container.
    /// @param args All the parameters. Can contain both l-values and r-values.
    template <typename... TParams>
    inline void push_front(TParams&&... args);

    /// @brief Pop element at the back.
    /// @details Forwards request to popBack() function of the underlying
    ///          container.
    inline void pop_back();

    /// @brief Pop element at the front.
    /// @details Forwards request to popFront() function of the underlying
    ///          container.
    inline void pop_front();

    /// @brief Retrieve reference to the last element.
    /// @details Forward the request to back() function of the underlying
    ///          container.
    inline reference back();

    /// @brief Const version of the back() function.
    inline const_reference back() const;

    /// @brief Retrieve reference to the first element.
    /// @details Forward the request to front() function of the underlying
    ///          container.
    inline reference front();

    /// @brief Const version of the front() function.
    inline const_reference front() const;

    /// @brief Returns size of the container.
    /// @details Forwards the request to size() function of the underlying
    ///          container.
    inline size_type size() const;

    /// @brief Returns whether the container is empty.
    /// @details Forwards the request to size() function of the underlying
    ///          container.
    /// @return true in case the container is empty, false otherwise.
    inline bool empty() const;

    /// @brief Clears the container.
    /// @details Forwards the request to clear() function of the underlying
    ///          container.
    inline void clear();

    /// @brief Returns the capacity of the container.
    /// @details Forwards the request to capacity() function of the underlying
    ///          container.
    /// @return Unsigned value - capacity of the underlying container.
    inline size_type capacity() const;

    /// @brief Iterator to the beginning
    /// @details Forwards the request to begin() function of the underlying
    ///          container.
    /// @return Iterator to the beginning.
    inline iterator begin();

    /// @brief Const version of the begin()
    inline const_iterator begin() const;

    /// @brief Const iterator to the beginning
    /// @details Forwards the request to cbegin() function of the underlying
    ///          container.
    /// @return Const iterator to the beginning.
    inline const_iterator cbegin() const;

    /// @brief Reverse iterator to the reverse beginning.
    /// @details Forwards the request to rbegin() function of the underlying
    ///          container.
    /// @return Reverse iterator to the reverse beginning.
    inline reverse_iterator rbegin();

    /// @brief Const version of the rbegin()
    inline const_reverse_iterator rbegin() const;

    /// @brief Const reverse iterator to the reverse beginning.
    /// @details Forwards the request to crbegin() function of the underlying
    ///          container.
    /// @return Const reverse iterator to the reverse beginning.
    inline const_reverse_iterator crbegin() const;

    /// @brief Iterator to the end.
    /// @details Forwards the request to end() function of the underlying
    ///          container.
    /// @return Iterator to the end.
    inline iterator end();

    /// @brief Const version of the end().
    inline const_iterator end() const;

    /// @brief Const iterator to the end.
    /// @details Forwards the request to cend() function of the underlying
    ///          container.
    /// @return Const iterator to the end.
    inline const_iterator cend() const;

    /// @brief Reverse iterator to the reverse end.
    /// @details Forwards the request to rend() function of the underlying
    ///          container.
    /// @return Reverse iterator to the reverse end.
    inline reverse_iterator rend();

    /// @brief Const version of the rend().
    inline const_reverse_iterator rend() const;

    /// @brief Const reverse iterator to the reverse end.
    /// @details Forwards the request to crend() function of the underlying
    ///          container.
    /// @return Const reverse iterator to the reverse end.
    inline const_reverse_iterator crend() const;

private:
    TContainer container_; ///< Underlying container
};

/// @}

// Implementation part

template <typename TContainer>
template <typename... TParams>
inline
StaticQueueStlAdapter<TContainer>::StaticQueueStlAdapter(TParams&&... args)
    : container_(std::forward<TParams>(args)...)
{
}


template <typename TContainer>
inline
StaticQueueStlAdapter<TContainer>::operator TContainer&()
{
    return container_;
}

template <typename TContainer>
inline
StaticQueueStlAdapter<TContainer>&
StaticQueueStlAdapter<TContainer>::operator=(const StaticQueueStlAdapter& other)
{
    container_ = other.container_;
    return *this;
}

template <typename TContainer>
inline
StaticQueueStlAdapter<TContainer>&
StaticQueueStlAdapter<TContainer>::operator=(StaticQueueStlAdapter&& other)
{
    container_ = std::move(other.container_);
    return *this;
}

template <typename TContainer>
inline
StaticQueueStlAdapter<TContainer>&
StaticQueueStlAdapter<TContainer>::operator=(const TContainer& other)
{
    container_ = other;
    return *this;
}

template <typename TContainer>
inline
StaticQueueStlAdapter<TContainer>&
StaticQueueStlAdapter<TContainer>::operator=(TContainer&& other)
{
    container_ = std::move(other);
    return *this;
}

template <typename TContainer>
inline
TContainer& StaticQueueStlAdapter<TContainer>::container()
{
    return container_;
}

template <typename TContainer>
inline
const TContainer& StaticQueueStlAdapter<TContainer>::container() const
{
    return container_;
}

template <typename TContainer>
template <typename... TParams>
inline
void StaticQueueStlAdapter<TContainer>::push_back(TParams&&... args)
{
    container_.pushBack(std::forward<TParams>(args)...);
}

template <typename TContainer>
template <typename... TParams>
inline
void StaticQueueStlAdapter<TContainer>::push_front(TParams&&... args)
{
    container_.pushFront(std::forward<TParams>(args)...);
}

template <typename TContainer>
inline
void StaticQueueStlAdapter<TContainer>::pop_back()
{
    container_.popBack();
}

template <typename TContainer>
inline
void StaticQueueStlAdapter<TContainer>::pop_front()
{
    container_.popFront();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::reference
StaticQueueStlAdapter<TContainer>::back()
{
    return container_.back();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::const_reference
StaticQueueStlAdapter<TContainer>::back() const
{
    return container_.back();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::reference
StaticQueueStlAdapter<TContainer>::front()
{
    return container_.front();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::const_reference
StaticQueueStlAdapter<TContainer>::front() const
{
    return container_.front();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::size_type
StaticQueueStlAdapter<TContainer>::size() const
{
    return container_.size();
}

template <typename TContainer>
inline
bool StaticQueueStlAdapter<TContainer>::empty() const
{
    return container_.isEmpty();
}

template <typename TContainer>
inline
void StaticQueueStlAdapter<TContainer>::clear()
{
    return container_.clear();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::size_type
StaticQueueStlAdapter<TContainer>::capacity() const
{
    return container_.capacity();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::iterator
StaticQueueStlAdapter<TContainer>::begin()
{
    return container_.lbegin();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::const_iterator
StaticQueueStlAdapter<TContainer>::begin() const
{
    return container_.lbegin();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::const_iterator
StaticQueueStlAdapter<TContainer>::cbegin() const
{
    return container_.clbegin();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::reverse_iterator
StaticQueueStlAdapter<TContainer>::rbegin()
{
    return container_.rlbegin();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::const_reverse_iterator
StaticQueueStlAdapter<TContainer>::rbegin() const
{
    return container_.rlbegin();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::const_reverse_iterator
StaticQueueStlAdapter<TContainer>::crbegin() const
{
    return container_.crlbegin();
}


template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::iterator
StaticQueueStlAdapter<TContainer>::end()
{
    return container_.lend();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::const_iterator
StaticQueueStlAdapter<TContainer>::end() const
{
    return container_.lend();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::const_iterator
StaticQueueStlAdapter<TContainer>::cend() const
{
    return container_.clend();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::reverse_iterator
StaticQueueStlAdapter<TContainer>::rend()
{
    return container_.rlend();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::const_reverse_iterator
StaticQueueStlAdapter<TContainer>::rend() const
{
    return container_.rlend();
}

template <typename TContainer>
inline
typename StaticQueueStlAdapter<TContainer>::const_reverse_iterator
StaticQueueStlAdapter<TContainer>::crend() const
{
    return container_.crlend();
}

}  // namespace container

}  // namespace embxx
