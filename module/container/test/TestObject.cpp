//
// Copyright 2012 (C). Alex Robenko. All rights reserved.
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

#include "TestObject.h"

#include <cstdlib>
#include <string>
#include <cassert>

#include "cxxtest/TestSuite.h"

std::size_t TestObject::objectCount_ = 0;
std::size_t TestObject::moveConstructCount_ = 0;
std::size_t TestObject::copyConstructCount_= 0;
std::size_t TestObject::moveAssignCount_ = 0;
std::size_t TestObject::copyAssignCount_ = 0;


TestObject::TestObject()
    : byte_(static_cast<std::uint8_t>(rand())),
      dword_(static_cast<std::uint32_t>(rand())),
      word_(static_cast<std::int16_t>(rand())),
      long_(static_cast<std::int64_t>(rand()) * rand()),
      floatPart_(new FloatPart),
      constructed_(true)
{
    ++objectCount_;
    TS_TRACE((std::string("Created new object, count = ") +
        std::to_string(objectCount_)).c_str());
}

TestObject::~TestObject()
{
    constructed_ = false;
    --objectCount_;
    TS_TRACE((std::string("Destructing object, count = ") +
        std::to_string(objectCount_)).c_str());
}

TestObject::TestObject(const TestObject& obj)
    : byte_(obj.byte_),
      dword_(obj.dword_),
      word_(obj.word_),
      long_(obj.long_),
      floatPart_(new FloatPart(*obj.floatPart_)),
      constructed_(true)
{
    assert(obj.constructed_);
    ++copyConstructCount_;
    ++objectCount_;
    TS_TRACE((std::string("Copy constructing object, count = ") +
        std::to_string(objectCount_)).c_str());
}

TestObject::TestObject(TestObject&& obj)
    : byte_(obj.byte_),
      dword_(obj.dword_),
      word_(obj.word_),
      long_(obj.long_),
      floatPart_(std::move(obj.floatPart_)),
      constructed_(true)
{
    assert(obj.constructed_);
    ++moveConstructCount_;
    ++objectCount_;
    TS_TRACE((std::string("Move constructing object, count = ") +
        std::to_string(objectCount_)).c_str());
}

TestObject&
TestObject::operator=(const TestObject& obj)
{
    assert(obj.constructed_);
    assert(constructed_);
    if (this == &obj) {
        return *this;
    }

    byte_ = obj.byte_;
    dword_ = obj.dword_;
    word_ = obj.word_;
    long_ = obj.long_;
    floatPart_.reset(new FloatPart(*obj.floatPart_));
    ++copyAssignCount_;
    return *this;
}

TestObject&
TestObject::operator=(TestObject&& obj)
{
    assert(obj.constructed_);
    assert(constructed_);

    if (this == &obj) {
        return *this;
    }

    byte_ = obj.byte_;
    dword_ = obj.dword_;
    word_ = obj.word_;
    long_ = obj.long_;
    floatPart_ = std::move(obj.floatPart_);
    ++moveAssignCount_;
    return *this;
}

TestObject::FloatPart::FloatPart()
    : float_(static_cast<float>(rand())/rand()),
      double_(static_cast<double>(rand())/rand())
{
}

bool TestObject::operator==(const TestObject& obj) const
{
    assert(obj.constructed_);
    assert(constructed_);

    if ((byte_ != obj.byte_) ||
        (dword_ != obj.dword_) ||
        (word_ != obj.word_) ||
        (long_ != obj.long_)) {
        return false;
    }

    if ((!floatPart_) && (!obj.floatPart_)) {
        return true;
    }

    if ((!floatPart_) || (!obj.floatPart_)) {
        return false;
    }

    return ((floatPart_->float_ == obj.floatPart_->float_) &&
            (floatPart_->double_ == obj.floatPart_->double_));
}

bool TestObject::operator<(const TestObject& obj) const
{
    assert(obj.constructed_);
    assert(constructed_);

    if ((obj.byte_ <= byte_) ||
        (obj.dword_ <= dword_) ||
        (obj.word_ <= word_) ||
        (obj.long_ <= long_)) {
        return false;
    }

    if (!obj.floatPart_) {
        return false;
    }

    if (!floatPart_) {
        return true;
    }

    return ((floatPart_->float_ < obj.floatPart_->float_) &&
            (floatPart_->double_ < obj.floatPart_->double_));
}

bool TestObject::isValid() const
{
    return constructed_ && static_cast<bool>(floatPart_);
}

bool TestObject::supportsMoveSemantics()
{
    return true;
}

std::size_t TestObject::getObjectCount()
{
    return objectCount_;
}

std::size_t TestObject::getMoveConstructCount()
{
    return moveConstructCount_;
}

std::size_t TestObject::getCopyConstructCount()
{
    return copyConstructCount_;
}

std::size_t TestObject::getMoveAssignCount()
{
    return moveAssignCount_;
}

std::size_t TestObject::getCopyAssignCount()
{
    return copyAssignCount_;
}

void TestObject::clearMoveConstructCount()
{
    moveConstructCount_ = 0;
}

void TestObject::clearCopyConstructCount()
{
    copyConstructCount_ = 0;
}

void TestObject::clearMoveAssignCount()
{
    moveAssignCount_ = 0;
}

void TestObject::clearCopyAssignCount()
{
    copyAssignCount_ = 0;
}

void TestObject::clearAllCopyMoveCounts()
{
    clearMoveConstructCount();
    clearCopyConstructCount();
    clearMoveAssignCount();
    clearCopyAssignCount();
}

std::string TestObject::toString() const
{
    assert(constructed_);
    std::string str;
    str += "Byte=";
    str += std::to_string(byte_);
    str += "; Dword=";
    str += std::to_string(dword_);
    str += "; Word=";
    str += std::to_string(word_);
    str += "; Long=";
    str += std::to_string(long_);
    if (floatPart_) {
        str += "; Float=";
        str += std::to_string(floatPart_->float_);
        str += "; Double=";
        str += std::to_string(floatPart_->double_);
    }
    return std::move(str);
}
