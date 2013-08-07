#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

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

    static std::size_t objectCount_;
    static std::size_t moveConstructCount_;
    static std::size_t copyConstructCount_;
    static std::size_t moveAssignCount_;
    static std::size_t copyAssignCount_;

};

