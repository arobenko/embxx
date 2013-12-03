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
#include <memory>
#include <iterator>

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
class Message1 :
    public embxx::comms::MessageBase<
        MessageType1,
        TestMessageBase<TTraits>,
        Message1<TTraits> >
{
    typedef
        embxx::comms::MessageBase<
            MessageType1,
            TestMessageBase<TTraits>,
            Message1<TTraits> > Base;
public:
    typedef std::uint16_t ValueType;
    typedef typename Base::ReadIterator ReadIterator;
    typedef typename Base::WriteIterator WriteIterator;

    Message1() : value_(0) {};

    virtual ~Message1() = default;

    ValueType getValue() const { return value_; }
    void setValue(ValueType value) { value_ = value; }

protected:
    virtual embxx::comms::ErrorStatus readImpl(ReadIterator& iter, std::size_t size)
    {
        TS_ASSERT_LESS_THAN_EQUALS(sizeof(value_), size);
        value_ = this->template readData<ValueType>(iter);
        return embxx::comms::ErrorStatus::Success;
    }

    virtual embxx::comms::ErrorStatus writeImpl(WriteIterator& iter, std::size_t size) const
    {
        TS_ASSERT_LESS_THAN_EQUALS(sizeof(value_), size);
        this->writeData(value_, iter);
        return embxx::comms::ErrorStatus::Success;
    }

    virtual std::size_t lengthImpl() const
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

template <typename TTraits,
          typename TProtStack>
typename TProtStack::MsgPtr
readWriteMsgTest(
    TProtStack& stack,
    const char* const buf,
    std::size_t bufSize,
    embxx::comms::ErrorStatus expectedErrStatus)
{
    typedef typename TProtStack::MsgPtr MsgPtr;
    MsgPtr msg;
    auto readIter = buf;
    auto es = stack.read(msg, readIter, bufSize);
    TS_ASSERT_EQUALS(es, expectedErrStatus);
    TS_ASSERT(!msg);
    return std::move(msg);
}

template <typename TTraits,
          typename TProtStack>
typename TProtStack::MsgPtr
readNotEnoughDataMsgTest(
    TProtStack& stack,
    const char* const buf,
    std::size_t bufSize,
    std::size_t expectedMissingSize)
{
    typedef typename TProtStack::MsgPtr MsgPtr;
    MsgPtr msg;
    auto readIter = buf;
    auto missingSize = std::numeric_limits<std::size_t>::max();
    auto es = stack.read(msg, readIter, bufSize, &missingSize);
    TS_ASSERT_EQUALS(es, embxx::comms::ErrorStatus::NotEnoughData);
    TS_ASSERT_EQUALS(missingSize, expectedMissingSize);
    TS_ASSERT(!msg);
    return std::move(msg);
}

template <typename TTraits,
          template<class> class TProtStack>
typename TProtStack<TTraits>::Type::MsgPtr
readWriteMsgTest(
    const char* const buf,
    std::size_t bufSize,
    embxx::comms::ErrorStatus expectedErrStatus)
{
    typedef typename TProtStack<TTraits>::Type ProtStack;
    ProtStack stack;
    return readWriteMsgTest<TTraits>(stack, buf, bufSize, expectedErrStatus);
}

template <typename TTraits,
          template<class> class TMessage,
          typename TProtStack>
TMessage<TTraits>
successfulReadWriteMsgTest(
    TProtStack& stack,
    const char* const buf,
    std::size_t bufSize)
{
    typedef TMessage<TTraits> ExpectedMsg;
    typedef typename TProtStack::MsgPtr MsgPtr;
    MsgPtr msg;
    auto readIter = buf;
    auto es = stack.read(msg, readIter, bufSize);
    TS_ASSERT_EQUALS(es, embxx::comms::ErrorStatus::Success);
    TS_ASSERT(msg);

    MessageHandler<TTraits> handler;
    msg->dispatch(handler);
    TS_ASSERT(handler.countCaught_ > 0);
    TS_ASSERT(handler.countUncaught_ == 0);

    auto actualBufSize = static_cast<std::size_t>(std::distance(buf, readIter));
    TS_ASSERT_EQUALS(actualBufSize, msg->length() + stack.length());
    std::unique_ptr<char []> outCheckBuf(new char[actualBufSize]);
    auto writeIter = &outCheckBuf[0];
    es = stack.write(*msg, writeIter, actualBufSize);
    TS_ASSERT_EQUALS(es, embxx::comms::ErrorStatus::Success);
    TS_ASSERT(std::equal(buf, buf + actualBufSize, &outCheckBuf[0]));

    auto castedMsg = dynamic_cast<ExpectedMsg*>(msg.get());
    return *castedMsg;
}

template <typename TTraits,
          template<class> class TMessage,
          template<class> class TProtStack>
TMessage<TTraits>
successfulReadWriteMsgTest(
    const char* const buf,
    std::size_t bufSize)
{
    typedef typename TProtStack<TTraits>::Type ProtStack;
    ProtStack stack;
    return successfulReadWriteMsgTest<TTraits, TMessage>(stack, buf, bufSize);
}

template <typename TTraits,
          template<class> class TMessage,
          template<class> class TProtStack>
TMessage<TTraits>
successfulReadWriteVectorMsgTest(
    const char* const buf,
    std::size_t bufSize)
{
    typedef typename TProtStack<TTraits>::Type ProtStack;
    typedef TMessage<TTraits> ExpectedMsg;

    ProtStack stack;
    typedef typename ProtStack::MsgPtr MsgPtr;
    MsgPtr msg;
    auto readIter = buf;
    auto es = stack.read(msg, readIter, bufSize);
    TS_ASSERT_EQUALS(es, embxx::comms::ErrorStatus::Success);
    TS_ASSERT(msg);

    MessageHandler<TTraits> handler;
    msg->dispatch(handler);
    TS_ASSERT(handler.countCaught_ > 0);
    TS_ASSERT(handler.countUncaught_ == 0);

    auto actualBufSize = static_cast<std::size_t>(std::distance(buf, readIter));
    std::vector<char> outCheckBuf;
    auto writeIter = std::back_inserter(outCheckBuf);
    es = stack.write(*msg, writeIter, actualBufSize);
    TS_ASSERT_EQUALS(es, embxx::comms::ErrorStatus::UpdateRequired);
    auto updateIter = &outCheckBuf[0];
    es = stack.update(updateIter, outCheckBuf.size());
    TS_ASSERT(std::equal(buf, buf + actualBufSize, &outCheckBuf[0]));

    auto castedMsg = dynamic_cast<ExpectedMsg*>(msg.get());
    return *castedMsg;
}

template <typename TTraits,
          template<class> class TMessage,
          typename TProtStack>
void writeReadMsgTest(
    TProtStack& stack,
    const TMessage<TTraits>& msg,
    char* const buf,
    std::size_t bufSize,
    embxx::comms::ErrorStatus expectedErrStatus,
    const char* expectedBuf = 0)
{
    typedef TMessage<TTraits> Message;
    auto writeIter = buf;
    embxx::comms::ErrorStatus es = stack.write(msg, writeIter, bufSize);
    TS_ASSERT_EQUALS(es, expectedErrStatus);
    if (es != embxx::comms::ErrorStatus::Success) {
        return;
    }

    TS_ASSERT(std::equal(&buf[0], &buf[0] + bufSize, &expectedBuf[0]));

    typedef typename TProtStack::MsgPtr MsgPtr;
    MsgPtr readMsgPtr;
    auto readIter = expectedBuf;
    es = stack.read(readMsgPtr, readIter, bufSize);
    TS_ASSERT_EQUALS(es, embxx::comms::ErrorStatus::Success);
    TS_ASSERT(readMsgPtr);

    auto castedMsg = dynamic_cast<Message*>(readMsgPtr.get());
    TS_ASSERT_EQUALS(*castedMsg, msg);
}

template <typename TTraits,
          template<class> class TMessage,
          template<class> class TProtStack>
void writeReadMsgTest(
    const TMessage<TTraits>& msg,
    char* const buf,
    std::size_t bufSize,
    embxx::comms::ErrorStatus expectedErrStatus,
    const char* expectedBuf = 0)
{
    typedef typename TProtStack<TTraits>::Type ProtStack;
    ProtStack stack;
    writeReadMsgTest<TTraits, TMessage>(stack, msg, buf, bufSize, expectedErrStatus, expectedBuf);
}
