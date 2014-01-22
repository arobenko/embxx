//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
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

namespace embxx
{

namespace io
{

namespace details
{

template <typename TValueType>
class ValueManipBase
{
public:
    typedef TValueType ValueType;
    ValueType value() const { return value_; }
protected:
    ValueManipBase(ValueType value) : value_(value) {}
private:
    ValueType value_;
};

}  // namespace details

/// @addtogroup io
/// @{

/// @brief Type of embxx::io::endl stream manipulator;
enum Endl
{
    endl ///< End of line stream manipulator
};

/// @brief Type of embxx::io::ends stream manipulator;
enum Ends
{
    ends ///< End of string stream manipulator
};

/// @brief Type of numeric base stream manipulator;
enum Base
{
    bin, ///< Binary numeric base stream manipulator
    oct, ///< Octal numeric base stream manipulator
    dec, ///< Decimal numeric base stream manipulator
    hex, ///< Hexadecimal numeric base stream manipulator
    Base_NumOfBases ///< Must be last
};

/// @brief Output stream manipulator class that sets the width of the next fields.
/// @details Use embxx::io::width() function to create object of this class.
class WidthManip : public details::ValueManipBase<std::size_t>
{
    typedef details::ValueManipBase<std::size_t> Base;
public:
    WidthManip(std::size_t value) : Base(value) {}
};

/// @brief Creates output stream manipulator object that sets width value of the stream
/// @details "stream << embxx::io::width(2)" is equivalent to "stream.width(2)".
inline
WidthManip width(std::size_t value)
{
    return WidthManip(value);
}

/// @brief Output stream manipulator class that sets the fill character.
/// @details Use embxx::io::fill() function to create object of this class.
template <typename T>
class FillManip : public details::ValueManipBase<T>
{
    typedef details::ValueManipBase<T> Base;
public:
    FillManip(std::size_t value) : Base(value) {}
};

/// @brief Creates output stream manipulator object that sets fill character of the stream.
/// @details "stream << embxx::io::fill('0')" is equivalent to "stream.fill('0')".
template <typename T>
inline
FillManip<T> fill(T value)
{
    return FillManip<T>(value);
}

/// @}


}  // namespace io

}  // namespace embxx


