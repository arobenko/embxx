//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//


#include <iostream>
#include <mutex>
#include <functional>
#include <thread>

#include "embxx/util/StreamLogger.h"
#include "embxx/util/log/LevelStringPrefixer.h"
#include "embxx/util/log/StreamableValueSuffixer.h"
#include "embxx/util/log/StreamFlushSuffixer.h"
#include "embxx/util/log/Locker.h"

typedef
        embxx::util::log::StreamFlushSuffixer<
            embxx::util::log::StreamableValueSuffixer<
                char,
                embxx::util::log::LevelStringPrefixer<
                    embxx::util::StreamLogger<embxx::util::log::Debug, std::ostream>
                >
            >
        > NonLockedLogger;

typedef
    embxx::util::log::Locker<
        std::mutex,
        NonLockedLogger
    > Logger;

template <typename TLogger>
void threadFunc(std::size_t tId, TLogger& logger)
{
    for (auto i = 0; i < 100; ++i) {
        SLOG(logger, embxx::util::log::Info, "Thread: " << tId << " ; Idx = " << i);
    }
}

int main(int argc, const char* argv[]) {
    static_cast<void>(argc);
    static_cast<void>(argv);

    NonLockedLogger logger1('\n', std::cout);

    // Output may be interleaved
    std::thread th1(&threadFunc<NonLockedLogger>, 1, std::ref(logger1));
    std::thread th2(&threadFunc<NonLockedLogger>, 2, std::ref(logger1));

    th1.join();
    th2.join();
    std::cout << "================================\n";

    // Output will NOT me interleaved
    std::mutex mutex;
    Logger logger2(mutex, '\n', std::cout);
    std::thread th3(&threadFunc<Logger>, 3, std::ref(logger2));
    std::thread th4(&threadFunc<Logger>, 4, std::ref(logger2));

    th3.join();
    th4.join();

    return 0;
}
