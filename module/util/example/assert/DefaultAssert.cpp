#include "embxx/util/Assert.h"

int main(int argc, const char* argv[]) {
    static_cast<void>(argc);
    static_cast<void>(argv);

    // Standard "assert" is activated in debug mode compilation
    // if NOSTDLIB is not defined
    GASSERT(false);
    return 0;
}
