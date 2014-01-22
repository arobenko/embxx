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

#include <deque>
#include <cstddef>
#include <list>
#include <functional>
#include <mutex>

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
class ReadFifo
{
public:
    typedef TCharType CharType;

    static const std::size_t FifoSize = TFifoSize;
    static const std::size_t FifoOpDelay = TFifoOpDelayMs;
    typedef std::deque<CharType> DataSeq;
    typedef std::list<CharType> Fifo;
    typedef std::function<void ()> ReadAvailableHandler;

    ReadFifo(boost::asio::io_service& io)
        : io_(io),
          timer_(io_)
    {
    }

    ~ReadFifo()
    {
        timer_.cancel();
    }

    void setDataToRead(const CharType* data, std::size_t size)
    {
        assert(data_.empty());
        data_.assign(data, data + size);
    }

    void setDataToRead(DataSeq&& data)
    {
        assert(data_.empty());
        data_ = std::move(data);
    }

    const DataSeq& getDataToRead() const
    {
        return data_;
    }

    template <typename TFunc>
    void setReadAvailableHandler(TFunc&& func)
    {
        handler_ = std::forward<TFunc>(func);
    }

    bool complete() const
    {
        return fifo_.empty() && data_.empty();
    }

    void startRead()
    {
        timer_.cancel();
        io_.post(std::bind(&ReadFifo::charArrived, this));
    }

    void stopRead()
    {
        timer_.cancel();
    }

    void clear()
    {
        data_.clear();
        fifo_.clear();
        timer_.cancel();
    }

    bool canRead() const
    {
        return !fifo_.empty();
    }

    CharType read()
    {
        assert(canRead());
        auto retval = fifo_.front();
        fifo_.pop_front();
        return retval;
    }

private:

    void scheduleNextCharArrived()
    {
        timer_.expires_from_now(boost::posix_time::milliseconds(FifoOpDelay));
        timer_.async_wait(
            [this](const boost::system::error_code& ec)
            {
                if (ec == boost::asio::error::operation_aborted) {
                    return;
                }

                assert(!ec);
                charArrived();
            });
    }

    void charArrived()
    {
        if (data_.empty()) {
            return;
        }

        scheduleNextCharArrived();

        if (FifoSize <= fifo_.size()) {
            return;
        }

        fifo_.push_back(data_.front());
        data_.pop_front();
        if (canRead()) {
            // First character
            assert(handler_);
            handler_();
        }
    }

    boost::asio::io_service& io_;
    boost::asio::deadline_timer timer_;

    ReadAvailableHandler handler_;
    DataSeq data_;
    Fifo fifo_;
};

}  // namespace test

}  // namespace device

}  // namespace embxx




