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

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

class TestObject
{
public:
    TestObject();
    ~TestObject();

    TestObject(const TestObject& obj);
    TestObject(TestObject&& obj);
    TestObject& operator=(const TestObject& obj);
    TestObject& operator=(TestObject&& obj);

    bool operator==(const TestObject& obj) const;
    bool operator<(const TestObject& obj) const;

    bool isValid() const;
    static bool supportsMoveSemantics();

    static std::size_t getObjectCount();
    static std::size_t getMoveConstructCount();
    static std::size_t getCopyConstructCount();
    static std::size_t getMoveAssignCount();
    static std::size_t getCopyAssignCount();

    static void clearMoveConstructCount();
    static void clearCopyConstructCount();
    static void clearMoveAssignCount();
    static void clearCopyAssignCount();
    static void clearAllCopyMoveCounts();

    std::string toString() const;

private:
    struct FloatPart
    {
        FloatPart();
        float float_;
        double double_;
    };
    std::uint8_t byte_;
    std::uint32_t dword_;
    std::int16_t word_;
    std::int16_t long_;
    std::unique_ptr<FloatPart> floatPart_;
    bool constructed_;

    static std::size_t objectCount_;
    static std::size_t moveConstructCount_;
    static std::size_t copyConstructCount_;
    static std::size_t moveAssignCount_;
    static std::size_t copyAssignCount_;

};

