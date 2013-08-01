#include <iostream>
#include <memory>

#include "embxx/util/ScopeGuard.h"

void guardFunc(int param)
{
    std::cout << "Guard with param: " << param << " is activated\n";
}

int main(int argc, const char* argv[]) {
    static_cast<void>(argc);
    static_cast<void>(argv);

    auto guard1 = embxx::util::makeScopeGuard(
        []()
        {
            std::cout << "guard1 is activated\n";
        });

    auto guard2 = embxx::util::makeScopeGuard(
        []()
        {
            std::cout << "guard2 is activated\n";
        });

    auto guard3 = embxx::util::makeScopeGuard(
        []()
        {
            std::cout << "guard3 is activated\n";
        });

    auto guard4 = embxx::util::makeScopeGuard(
        std::bind(&guardFunc, 4));

    auto guard5 = embxx::util::makeScopeGuard(
            std::bind(&guardFunc, 5));

    auto guard6 = embxx::util::makeScopeGuard(
            std::bind(&guardFunc, 6));

    guard2.release();
    guard5.release();
    return 0;
}
