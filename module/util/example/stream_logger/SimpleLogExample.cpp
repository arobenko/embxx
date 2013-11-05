//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

#include <iostream>

#include "embxx/util/StreamLogger.h"
#include "embxx/util/log/LevelStringPrefixer.h"
#include "embxx/util/log/StreamableValueSuffixer.h"
#include "embxx/util/log/StreamFlushSuffixer.h"

namespace log = embxx::util::log;

int main(int argc, const char* argv[]) {
    static_cast<void>(argc);
    static_cast<void>(argv);

    typedef embxx::util::StreamLogger<log::Warning, std::ostream> SimpleLogger;

    SimpleLogger simpleLogger(std::cout);

    SLOG(simpleLogger, log::Trace, "This string is not visible\n");
    SLOG(simpleLogger, log::Error, "Non formatted error string\n");

    typedef embxx::util::log::StreamFlushSuffixer<
        embxx::util::log::StreamableValueSuffixer<
            char,

            embxx::util::log::LevelStringPrefixer<
                embxx::util::StreamLogger<log::Debug, std::ostream> > > > ComplexLogger;

    ComplexLogger complexLogger('\n', std::cout);
    SLOG(complexLogger, log::Trace, "Not visible formatted string");
    SLOG(complexLogger, log::Info, "Information " << "text" );
    SLOG(complexLogger, log::Warning, "Warning " << "text");

    return 0;
}
