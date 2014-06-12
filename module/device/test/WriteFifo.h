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

#include <vector>
#include <cstddef>
#include <list>
#include <functional>
#include <mutex>
#include <algorithm>
#include <iterator>

#include <boost/asio.hpp>
#include <boost/date_time.hpp>

namespace embxx
{

namespace device
{

namespace test
{

template <typename TCharType,
          std::size_t TFifoSize,
          std::size_t TFifoOpDelayMs>
class WriteFifo
{
public:
    typedef TCharType CharType;

    static const std::size_t FifoSize = TFifoSize;
    static const std::size_t FifoOpDelay = TFifoOpDelayMs;
    typedef std::vector<CharType> DataSeq;
    typedef std::list<CharType> Fifo;
    typedef std::function<void ()> WriteAvailableHandler;

    WriteFifo(boost::asio::io_service& io)
        : io_(io),
          timer_(io_),
          writing_(false)
    {
    }

    ~WriteFifo()
    {
        timer_.cancel();
    }

    const DataSeq& getWrittenData() const
    {
        return data_;
    }

    template <typename TFunc>
    void setWriteAvailableHandler(TFunc&& func)
    {
        handler_ = std::forward<TFunc>(func);
    }

    bool empty() const
    {
        return data_.empty() && fifo_.empty();
    }

    bool complete() const
    {
        return fifo_.empty();
    }

    void startWrite()
    {
        timer_.cancel();
        io_.post(std::bind(&WriteFifo::charWritten, this));
    }

    void stopWrite()
    {
        timer_.cancel();
    }

    void clear()
    {
        data_.clear();
        fifo_.clear();
        timer_.cancel();
    }

    bool canWrite() const
    {
        return fifo_.size() < FifoSize;
    }

    void write(CharType value)
    {
        assert(canWrite());
        fifo_.push_back(value);
        if (!writing_) {
            scheduleNextCharWritten();
        }
    }

    void flush()
    {
        timer_.cancel();
        std::copy(fifo_.begin(), fifo_.end(), std::back_inserter(data_));
        fifo_.clear();
        writing_ = false;
    }

private:

    void scheduleNextCharWritten()
    {
        assert(!writing_);
        writing_ = true;
        timer_.expires_from_now(boost::posix_time::milliseconds(FifoOpDelay));
        timer_.async_wait(
            [this](const boost::system::error_code& ec)
            {
                if (ec == boost::asio::error::operation_aborted) {
                    return;
                }

                assert(!ec);
                charWritten();
            });
    }

    void charWritten()
    {
        writing_ = false;
        if (!fifo_.empty()) {
            data_.push_back(fifo_.front());
            fifo_.pop_front();
        }

        if (!fifo_.empty()) {
            scheduleNextCharWritten();
        }

        if (canWrite() && handler_) {
            handler_();
        }
    }

    boost::asio::io_service& io_;
    boost::asio::deadline_timer timer_;

    WriteAvailableHandler handler_;
    DataSeq data_;
    Fifo fifo_;
    bool writing_;
};

}  // namespace test

}  // namespace device

}  // namespace embxx




