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

#include <condition_variable>

namespace embxx
{

namespace device
{

namespace test
{

class EventLoopCond
{
public:
    EventLoopCond() : notified_(false) {}

    template <typename TLock>
    void wait(TLock& lock)
    {
        if (!notified_) {
            cond_.wait(lock);
        }
        notified_ = false;
    }

    void notify()
    {
        notified_ = true;
        cond_.notify_all();
    }

private:
    std::condition_variable_any cond_;
    bool notified_;
};


}  // namespace test

}  // namespace device

}  // namespace embxx


