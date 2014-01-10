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

#include <thread>

#include <boost/asio.hpp>

#include "cxxtest/TestSuite.h"

namespace embxx
{

namespace device
{

namespace test
{

class TestDevice
{
public:
    TestDevice()
        : work_(io_)
    {
        thread_.reset(
            new std::thread(
                [this]()
                {
                    try {
                        io_.run();
                    }
                    catch (const std::exception& e)
                    {
                        TS_FAIL(e.what());
                        assert(!"Mustn't happen");
                    }
                }));
    }

    ~TestDevice()
    {
        stopThread();
    }

protected:

    void stopThread()
    {
        if (thread_) {
            io_.stop();
            thread_->join();
            thread_.reset();
        }
    }

    boost::asio::io_service io_;
    boost::asio::io_service::work work_;
    std::unique_ptr<std::thread> thread_;
};

}  // namespace test

}  // namespace device

}  // namespace embxx

