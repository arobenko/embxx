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

#include <string>
#include <tuple>
#include <algorithm>

#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream_buffer.hpp>

#include "embxx/comms/Message.h"
#include "embxx/comms/MessageHandler.h"
#include "embxx/comms/field.h"

enum MessageType {
    MessageType1,
    UnusedValue1,
    MessageType2,
    UnusedValue2,
    UnusedValue3,
    MessageType3
};

template <typename TTraits>
class TestMessageHandler;

template <typename TTraits>
class TestMessageBase : public embxx::comms::Message<TestMessageHandler<TTraits>, TTraits>
{
    typedef embxx::comms::Message<TestMessageHandler<TTraits>, TTraits> Base;
public:
    virtual ~TestMessageBase() {}

    const std::string& getName() const
    {
        return this->getNameImpl();
    }

private:
    virtual const std::string& getNameImpl() const = 0;
};

template <typename TTraits>
class Message1 : public embxx::comms::MessageBase<MessageType1,
                                           TestMessageBase<TTraits>,
                                           Message1<TTraits> >
{
public:
    typedef std::uint16_t ValueType;

    Message1() : value_(0) {};

    virtual ~Message1() = default;

    ValueType getValue() const { return value_; }
    void setValue(ValueType value) { value_ = value; }

protected:
    virtual embxx::comms::ErrorStatus readImpl(std::streambuf& buf, std::size_t size)
    {
        TS_ASSERT_LESS_THAN_EQUALS(sizeof(value_), size);
        value_ = this->template getData<ValueType>(buf);
        return embxx::comms::ErrorStatus::Success;
    }

    virtual embxx::comms::ErrorStatus writeImpl(std::streambuf& buf, std::size_t size) const
    {
        TS_ASSERT_LESS_THAN_EQUALS(sizeof(value_), size);
        this->putData(value_, buf);
        return embxx::comms::ErrorStatus::Success;
    }

    virtual std::size_t getDataSizeImpl() const
    {
        return sizeof(value_);
    }

    virtual const std::string& getNameImpl() const
    {
        static const std::string str("Message1");
        return str;
    }

private:
    ValueType value_;
};

template <typename TTraits>
bool operator==(const Message1<TTraits> msg1, const Message1<TTraits> msg2)
{
    return msg1.getValue() == msg2.getValue();
}

template <typename TTraits>
class Message2 : public embxx::comms::EmptyBodyMessage<MessageType2,
                                                TestMessageBase<TTraits>,
                                                Message2<TTraits> >
{
public:
    virtual ~Message2() = default;

    virtual const std::string& getNameImpl() const
    {
        static const std::string str("Message2");
        return str;
    }

};

template <typename TTraits>
bool operator==(const Message2<TTraits> msg1, const Message2<TTraits> msg2)
{
    static_cast<void>(msg1);
    static_cast<void>(msg2);

    return true;
}

template <typename TTraits>
struct Message3Fields
{
    typedef embxx::comms::field::BasicIntValue<std::uint32_t, TTraits> Field1;
    typedef embxx::comms::field::BasicIntValue<std::int16_t, TTraits, 1> Field2;
    typedef embxx::comms::field::BitmaskValue<2, TTraits> Field3;
    typedef embxx::comms::field::BitmaskValue<3, TTraits> Field4;

    typedef std::tuple<
        Field1,
        Field2,
        Field3,
        Field4
    > Type;
};

template <typename TTraits>
class Message3 : public embxx::comms::MetaMessageBase<MessageType3,
                                           TestMessageBase<TTraits>,
                                           Message3<TTraits>,
                                           typename Message3Fields<TTraits>::Type >
{
public:
    virtual ~Message3() {}

protected:

    virtual const std::string& getNameImpl() const
    {
        static const std::string str("Message3");
        return str;
    }
};

template <typename TTraits>
bool operator==(const Message3<TTraits> msg1, const Message3<TTraits> msg2)
{
    return msg1.getFields() == msg2.getFields();
}


template <typename TTraits>
struct AllMessages
{
    typedef std::tuple<
        Message1<TTraits>,
        Message2<TTraits>,
        Message3<TTraits> > Type;
};

template <typename TTraits>
struct TestMessageHandler : public embxx::comms::MessageHandler<TestMessageBase<TTraits>, typename AllMessages<TTraits>::Type >
{
};

template <typename TTraits>
struct MessageHandler : public TestMessageHandler<TTraits>
{
    virtual ~MessageHandler() {}

    virtual void handleMessage(Message1<TTraits>& msg)
    {
        static_cast<void>(msg);
        ++countCaught_;
    }

    virtual void handleMessage(Message2<TTraits>& msg)
    {
        static_cast<void>(msg);
        ++countCaught_;
    }

    virtual void handleMessage(Message3<TTraits>& msg)
    {
        static_cast<void>(msg);
        ++countCaught_;
    }

    virtual void handleMessage(embxx::comms::Message<TestMessageHandler<TTraits>, TTraits>& msg)
    {
        static_cast<void>(msg);
        ++countUncaught_;
    }

    MessageHandler() : countCaught_(0), countUncaught_(0) {}
    unsigned countCaught_;
    unsigned countUncaught_;
};

typedef boost::iostreams::array_sink Sink;
typedef boost::iostreams::array_source Source;
typedef boost::iostreams::array ArrayDevice;
typedef boost::iostreams::stream_buffer<Sink> OutputBuf;
typedef boost::iostreams::stream_buffer<Source> InputBuf;
typedef boost::iostreams::stream_buffer<ArrayDevice> InOutBuf;

template <typename TTraits,
          template<class> class TProtStack>
typename TProtStack<TTraits>::Type::MsgPtr
readWriteMsgTest(
    const char* buf,
    std::size_t bufSize,
    embxx::comms::ErrorStatus expectedErrStatus)
{
    typedef typename TProtStack<TTraits>::Type ProtStack;
    typedef typename ProtStack::MsgPtr MsgPtr;

    ProtStack stack;
    InputBuf inBuf(buf, bufSize);
    inBuf.pubseekpos(0, std::ios_base::in);

    MsgPtr msg;
    auto es = stack.read(msg, inBuf, bufSize);
    TS_ASSERT_EQUALS(es, expectedErrStatus);
    TS_ASSERT(!msg);
    return std::move(msg);
}

template <typename TTraits,
          template<class> class TMessage,
          template<class> class TProtStack>
TMessage<TTraits>
successfulReadWriteMsgTest(
    const char* buf,
    std::size_t bufSize)
{
    typedef typename TProtStack<TTraits>::Type ProtStack;
    typedef TMessage<TTraits> ExpectedMsg;

    ProtStack stack;
    InputBuf inBuf(buf, bufSize);
    inBuf.pubseekpos(0, std::ios_base::in);

    typedef typename ProtStack::MsgPtr MsgPtr;
    MsgPtr msg;
    auto es = stack.read(msg, inBuf, bufSize);
    TS_ASSERT_EQUALS(es, embxx::comms::ErrorStatus::Success);
    TS_ASSERT(msg);

    MessageHandler<TTraits> handler;
    msg->dispatch(handler);
    TS_ASSERT(handler.countCaught_ > 0);
    TS_ASSERT(handler.countUncaught_ == 0);

    auto actualBufSize = static_cast<std::size_t>(inBuf.pubseekoff(0, std::ios_base::cur, std::ios_base::in));
    std::unique_ptr<char []> outCheckBuf(new char[actualBufSize]);
    InOutBuf outBuf(&outCheckBuf[0], actualBufSize);
    outBuf.pubseekpos(0, std::ios_base::out);
    es = stack.write(*msg, outBuf, actualBufSize);
    TS_ASSERT_EQUALS(es, embxx::comms::ErrorStatus::Success);
    TS_ASSERT(std::equal(buf, buf + actualBufSize, &outCheckBuf[0]));

    auto castedMsg = dynamic_cast<ExpectedMsg*>(msg.get());
    return *castedMsg;
}

template <typename TTraits,
          template<class> class TMessage,
          template<class> class TProtStack>
void writeReadMsgTest(
    const TMessage<TTraits>& msg,
    char* buf,
    std::size_t bufSize,
    embxx::comms::ErrorStatus expectedErrStatus,
    const char* expectedBuf = 0)
{
    typedef typename TProtStack<TTraits>::Type ProtStack;
    typedef TMessage<TTraits> Message;

    ProtStack stack;
    InOutBuf outBuf(buf, bufSize);
    outBuf.pubseekpos(0, std::ios_base::out);
    embxx::comms::ErrorStatus es = stack.write(msg, outBuf, bufSize);
    TS_ASSERT_EQUALS(es, expectedErrStatus);
    if (es != embxx::comms::ErrorStatus::Success) {
        return;
    }

    TS_ASSERT(std::equal(&buf[0], &buf[0] + bufSize, &expectedBuf[0]));

    InputBuf inBuf(buf, bufSize);
    inBuf.pubseekpos(0, std::ios_base::in);
    typedef typename ProtStack::MsgPtr MsgPtr;
    MsgPtr readMsgPtr;
    es = stack.read(readMsgPtr, inBuf, bufSize);
    TS_ASSERT_EQUALS(es, embxx::comms::ErrorStatus::Success);
    TS_ASSERT(readMsgPtr);

    auto castedMsg = dynamic_cast<Message*>(readMsgPtr.get());
    TS_ASSERT_EQUALS(*castedMsg, msg);
}
