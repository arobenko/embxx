#include <stdexcept>
#include "embxx/util/Assert.h"

class ExceptionAssert : public embxx::util::Assert
{
public:
    virtual ~ExceptionAssert() {}
    virtual void fail(
            const char* expr,
            const char* file,
            unsigned int line,
            const char* function)
    {
        static_cast<void>(expr);
        static_cast<void>(file);
        static_cast<void>(line);
        static_cast<void>(function);

        throw std::runtime_error("Assertion failure");
    }
};

int main(int argc, const char* argv[]) {
    static_cast<void>(argc);
    static_cast<void>(argv);

    embxx::util::EnableAssert<ExceptionAssert> customAssert;

    // Uncaught exception here
    GASSERT(false);
    return 0;
}
