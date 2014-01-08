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

#include <array>
#include <functional>

#include "embxx/error/ErrorStatus.h"
#include "embxx/util/StaticFunction.h"
#include "embxx/util/ScopeGuard.h"
#include "embxx/container/StaticQueue.h"

#include "context.h"
#include "op_category.h"

namespace embxx
{

namespace device
{

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler = embxx::util::StaticFunction<void()>,
          typename TOpCompleteHandler = embxx::util::StaticFunction<void (const embxx::error::ErrorStatus&)> >
class DeviceOpQueue
{
public:
    typedef TDevice Device;
    static const std::size_t Size = TSize;
    typedef TCanDoOpHandler CanDoOpHandler;
    typedef TOpCompleteHandler OpCompleteHandler;

    typedef typename Device::CharType CharType;
    typedef typename Device::DeviceIdType DeviceIdType;
    typedef typename Device::OpCategory OpCategory;

    explicit DeviceOpQueue(Device& device);
    DeviceOpQueue(const DeviceOpQueue&) = default;
    ~DeviceOpQueue();

    DeviceOpQueue& operator=(const DeviceOpQueue&) = delete;

    Device& device();
    const Device& device() const;

    template <typename TFunc>
    void setCanReadHandler(DeviceIdType id, TFunc&& func);

    template <typename TFunc>
    void setCanWriteHandler(DeviceIdType id, TFunc&& func);

    template <typename TFunc>
    void setReadCompleteHandler(DeviceIdType id, TFunc&& func);

    template <typename TFunc>
    void setWriteCompleteHandler(DeviceIdType id, TFunc&& func);

    void startRead(
        DeviceIdType id,
        std::size_t length,
        const context::EventLoop& context);

    bool cancelRead(
        DeviceIdType id,
        const context::EventLoop& context);

    bool cancelRead(
        DeviceIdType id,
        const context::Interrupt& context);


    void startWrite(
        DeviceIdType id,
        std::size_t length,
        const context::EventLoop& context);

    bool cancelWrite(
        DeviceIdType id,
        const context::EventLoop& context);

    bool canRead(const context::Interrupt& context);
    bool canWrite(const context::Interrupt& context);
    CharType read(const context::Interrupt& context);
    void write(CharType value, const context::Interrupt& context);

private:
    struct DeviceInfo
    {
        DeviceInfo()
            : id_(0),
              valid_(false)
        {
        }

        bool isValid() const
        {
            return valid_;
        }

        void setValid(bool valid)
        {
            valid_ = valid;
        }

        DeviceIdType id_;
        CanDoOpHandler canReadHandler_;
        CanDoOpHandler canWriteHandler_;
        OpCompleteHandler readCompleteHandler_;
        OpCompleteHandler writeCompleteHandler_;
        bool valid_;
    };

    typedef std::array<DeviceInfo, Size> DeviceInfosArray;
    typedef typename DeviceInfosArray::iterator DeviceInfoIter;

    enum class OpType {
        Invalid,
        Read,
        Write
    };

    struct OpInfo
    {
        OpInfo() : id_(0), length_(0), op_(OpType::Invalid) {}
        OpInfo(DeviceIdType addr, std::size_t length, OpType op)
            : id_(addr), length_(length), op_(op) {}

        DeviceIdType id_;
        std::size_t length_;
        OpType op_;
    };

    void canReadHandler();
    void canWriteHandler();
    void readCompleteHandler(const embxx::error::ErrorStatus& err);
    void writeCompleteHandler(const embxx::error::ErrorStatus& err);

    static const std::size_t OpQueueSize =
        std::is_same<OpCategory, op_category::SequentialReadWrite>::value ?
            Size : Size * 2;
    typedef embxx::container::StaticQueue<OpInfo, OpQueueSize> OpQueue;
    typedef typename OpQueue::iterator OpQueueIterator;

    DeviceInfoIter findDeviceInfo(DeviceIdType addr);
    OpQueueIterator findOpInfo(const OpInfo& info);
    OpQueueIterator findOpInfo(
        const OpInfo& info,
        const op_category::SequentialReadWrite& category);
    OpQueueIterator findOpInfo(
        const OpInfo& info,
        const op_category::ParallelReadWrite& category);

    template <typename TContext>
    void startNextOp(OpType opType, const TContext& context);

    template <typename TContext>
    void startNextOp(
        OpType opType,
        const TContext& context,
        const op_category::SequentialReadWrite& category);

    template <typename TContext>
    void startNextOp(
        OpType opType,
        const TContext& context,
        const op_category::ParallelReadWrite& category);

    template <typename TContext>
    void startNextOp(
        OpType opType,
        DeviceIdType id,
        std::size_t length,
        const TContext& context);

    OpQueueIterator findNextOp(
        OpType opType,
        const op_category::SequentialReadWrite& category);
    OpQueueIterator findNextOp(
        OpType opType,
        const op_category::ParallelReadWrite& category);

    Device& device_;
    DeviceInfosArray infos_;
    DeviceInfoIter currReadDevInfo_;
    DeviceInfoIter currWriteDevInfo_;
    OpQueue opQueue_;
};

template <typename TDevice>
class DeviceOpQueue<TDevice, 1>
{
public:
    typedef TDevice Device;
    static const std::size_t Size = 1;

    typedef typename Device::CharType CharType;
    typedef typename Device::DeviceIdType DeviceIdType;
    typedef typename Device::OpCategory OpCategory;

    explicit DeviceOpQueue(Device& device);
    DeviceOpQueue(const DeviceOpQueue&) = default;
    ~DeviceOpQueue() = default;

    DeviceOpQueue& operator=(const DeviceOpQueue&) = delete;

    Device& device();
    const Device& device() const;

    template <typename TFunc>
    void setCanReadHandler(DeviceIdType id, TFunc&& func);

    template <typename TFunc>
    void setCanWriteHandler(DeviceIdType id, TFunc&& func);

    template <typename TFunc>
    void setReadCompleteHandler(DeviceIdType id, TFunc&& func);

    template <typename TFunc>
    void setWriteCompleteHandler(DeviceIdType id, TFunc&& func);

    template <typename... TArgs>
    void startRead(TArgs&&... args);

    template <typename... TArgs>
    bool cancelRead(
        DeviceIdType id,
        TArgs&&... args);

    template <typename... TArgs>
    void startWrite(TArgs&&... args);

    template <typename... TArgs>
    bool cancelWrite(
        DeviceIdType id,
        TArgs&&... args);

    template <typename... TArgs>
    bool canRead(TArgs&&... args);

    template <typename... TArgs>
    bool canWrite(TArgs&&... args);

    template <typename... TArgs>
    CharType read(TArgs&&... args);

    template <typename... TArgs>
    void write(TArgs&&... args);

private:
    Device& device_;
};


// Implementation
template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::DeviceOpQueue(
    Device& device)
    : device_(device),
      currReadDevInfo_(infos_.end()),
      currWriteDevInfo_(infos_.end())
{
    device_.setCanReadHandler(std::bind(&DeviceOpQueue::canReadHandler, this));
    device_.setCanWriteHandler(std::bind(&DeviceOpQueue::canWriteHandler, this));
    device_.setReadCompleteHandler(std::bind(&DeviceOpQueue::readCompleteHandler, this, std::placeholders::_1));
    device_.setWriteCompleteHandler(std::bind(&DeviceOpQueue::writeCompleteHandler, this, std::placeholders::_1));
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::~DeviceOpQueue()
{
    device_.setCanReadHandler(nullptr);
    device_.setCanWriteHandler(nullptr);
    device_.setReadCompleteHandler(nullptr);
    device_.setWriteCompleteHandler(nullptr);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
typename DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::Device&
DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::device()
{
    return device_;
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
const typename DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::Device&
DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::device() const
{
    return device_;
}


template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
template <typename TFunc>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::setCanReadHandler(
    DeviceIdType id,
    TFunc&& func)
{
    auto iter = findDeviceInfo(id);
    if (iter == infos_.end()) {
        GASSERT(!"Too many devices");
        return;
    }
    iter->canReadHandler_ = std::forward<TFunc>(func);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
template <typename TFunc>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::setCanWriteHandler(
    DeviceIdType id,
    TFunc&& func)
{
    auto iter = findDeviceInfo(id);
    if (iter == infos_.end()) {
        GASSERT(!"Too many devices");
        return;
    }
    iter->canWriteHandler_ = std::forward<TFunc>(func);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
template <typename TFunc>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::setReadCompleteHandler(
    DeviceIdType id,
    TFunc&& func)
{
    auto iter = findDeviceInfo(id);
    if (iter == infos_.end()) {
        GASSERT(!"Too many devices");
        return;
    }
    iter->readCompleteHandler_ = std::forward<TFunc>(func);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
template <typename TFunc>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::setWriteCompleteHandler(
    DeviceIdType id,
    TFunc&& func)
{
    auto iter = findDeviceInfo(id);
    if (iter == infos_.end()) {
        GASSERT(!"Too many devices");
        return;
    }
    iter->writeCompleteHandler_ = std::forward<TFunc>(func);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::startRead(
    DeviceIdType id,
    std::size_t length,
    const context::EventLoop& context)
{
    auto suspResult = device_.suspendRead(context);

    // If suspend is successful, the queue is not empty
    GASSERT((!suspResult) || (!opQueue_.empty()));

    OpInfo info(id, length, OpType::Read);
    GASSERT(findOpInfo(info) == opQueue_.end());

    opQueue_.push_back(std::move(info));
    if (suspResult) {
        device_.resumeRead(context);
        return;
    }

    startNextOp(OpType::Read, context);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
bool DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::cancelRead(
    DeviceIdType id,
    const context::EventLoop& context)
{
    auto suspResult = device_.suspendRead(context);

    // If suspend is successful, the queue is not empty
    GASSERT((!suspResult) || (!opQueue_.empty()));

    auto guard =
        embxx::util::makeScopeGuard(
            [this, suspResult, &context]()
            {
                if (suspResult) {
                    device_.resumeRead(context);
                }
            });

    OpInfo opInfo(id, 0, OpType::Read);
    auto iter = findOpInfo(opInfo);
    if (iter == opQueue_.end()) {
        return false; // resuming read
    }
    GASSERT(iter->op_ == OpType::Read);

    GASSERT(currReadDevInfo_ != infos_.end());
    if (id != currReadDevInfo_->id_) {
        GASSERT(opQueue_.front().id_ != id);
        opQueue_.erase(iter);
        return true; // resuming read
    }

    // Cancelling current op
    guard.release(); // No resume on return

    auto result = device_.cancelRead(context);
    GASSERT(result); // cancel must succeed
    opQueue_.erase(iter);
    startNextOp(OpType::Read, context);
    return result;
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::startWrite(
    DeviceIdType id,
    std::size_t length,
    const context::EventLoop& context)
{
    auto suspResult = device_.suspendWrite(context);

    // If suspend is successful, the queue is not empty
    GASSERT((!suspResult) || (!opQueue_.empty()));

    OpInfo info(id, length, OpType::Write);
    GASSERT(findOpInfo(info) == opQueue_.end());

    opQueue_.push_back(std::move(info));

    if (suspResult) {
        device_.resumeWrite(context);
        return;
    }

    startNextOp(OpType::Write, context);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
bool DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::cancelWrite(
    DeviceIdType id,
    const context::EventLoop& context)
{
    auto suspResult = device_.suspendWrite(context);

    // If suspend is successful, the queue is not empty
    GASSERT((!suspResult) || (!opQueue_.empty()));

    auto guard =
        embxx::util::makeScopeGuard(
            [this, suspResult, &context]()
            {
                if (suspResult) {
                    device_.resumeWrite(context);
                }
            });

    OpInfo opInfo(id, 0, OpType::Write);
    auto iter = findOpInfo(opInfo);
    if (iter == opQueue_.end()) {
        return false; // resuming write
    }
    GASSERT(iter->op_ == OpType::Write);

    GASSERT(currWriteDevInfo_ != infos_.end());
    if (id != currWriteDevInfo_->id_) {
        GASSERT(opQueue_.front().id_ != id);
        opQueue_.erase(iter);
        return true; // resuming write
    }

    // Cancelling current op
    guard.release(); // No resume on return

    auto result = device_.cancelWrite(context);
    GASSERT(result); // cancel must succeed
    opQueue_.erase(iter);
    startNextOp(OpType::Write, context);
    return result;
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
bool DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::cancelRead(
    DeviceIdType id,
    const context::Interrupt& context)
{
    static_cast<void>(id);
    GASSERT(currReadDevInfo_->id_ == id);
    bool result = device_.cancelRead(context);
    GASSERT(result == true);
    startNextOp(OpType::Read, context);
    return result;
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
bool DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::canRead(
    const context::Interrupt& context)
{
    return device_.canRead(context);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
bool DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::canWrite(
    const context::Interrupt& context)
{
    return device_.canWrite(context);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
typename DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::CharType
DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::read(
    const context::Interrupt& context)
{
    return device_.read(context);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::write(
    CharType value,
    const context::Interrupt& context)
{
    device_.write(value, context);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::canReadHandler()
{
    GASSERT(currReadDevInfo_ != infos_.end());
    GASSERT(currReadDevInfo_->canReadHandler_);
    currReadDevInfo_->canReadHandler_();
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::canWriteHandler()
{
    GASSERT(currWriteDevInfo_ != infos_.end());
    GASSERT(currWriteDevInfo_->canWriteHandler_);
    currWriteDevInfo_->canWriteHandler_();
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::readCompleteHandler(
    const embxx::error::ErrorStatus& err)
{
    GASSERT(currReadDevInfo_ != infos_.end());
    GASSERT(currReadDevInfo_->readCompleteHandler_);

    auto devInfo = currReadDevInfo_;
    auto currOp = findOpInfo(OpInfo(currReadDevInfo_->id_, 0, OpType::Read));
    GASSERT(currOp != opQueue_.end());
    opQueue_.erase(currOp);

    currReadDevInfo_ = infos_.end();
    startNextOp(OpType::Read, context::Interrupt());

    devInfo->readCompleteHandler_(err);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::writeCompleteHandler(
    const embxx::error::ErrorStatus& err)
{
    GASSERT(currWriteDevInfo_ != infos_.end());
    GASSERT(currWriteDevInfo_->writeCompleteHandler_);

    auto devInfo = currWriteDevInfo_;
    auto currOp = findOpInfo(OpInfo(currWriteDevInfo_->id_, 0, OpType::Write));
    GASSERT(currOp != opQueue_.end());
    opQueue_.erase(currOp);
    currWriteDevInfo_ = infos_.end();
    startNextOp(OpType::Write, context::Interrupt());

    devInfo->writeCompleteHandler_(err);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
typename DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::DeviceInfoIter
DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::findDeviceInfo(
    DeviceIdType id)
{
    auto iter = std::find_if(infos_.begin(), infos_.end(),
        [id](const DeviceInfo& info) -> bool
        {
            return ((!info.isValid()) || (info.id_ == id));
        });

    if (iter == infos_.end()) {
        return iter;
    }

    if (!iter->isValid()) {
        iter->setValid(true);
        iter->id_ = id;
    }
    return iter;
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
typename DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::OpQueueIterator
DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::findOpInfo(
    const OpInfo& info)
{
    return findOpInfo(info, OpCategory());
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
typename DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::OpQueueIterator
DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::findOpInfo(
    const OpInfo& info,
    const op_category::SequentialReadWrite& category)
{
    static_cast<void>(category);
    return std::find_if(opQueue_.begin(), opQueue_.end(),
        [&info](const OpInfo& elem) -> bool
        {
            return elem.id_ == info.id_;
        });
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
typename DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::OpQueueIterator
DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::findOpInfo(
    const OpInfo& info,
    const op_category::ParallelReadWrite& category)
{
    static_cast<void>(category);
    return std::find_if(opQueue_.begin(), opQueue_.end(),
        [&info](const OpInfo& elem) -> bool
        {
            return (elem.id_ == info.id_) && (elem.op_ == info.op_);
        });
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
template <typename TContext>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::startNextOp(
    OpType opType,
    const TContext& context)
{
    startNextOp(opType, context, OpCategory());
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
template <typename TContext>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::startNextOp(
    OpType opType,
    const TContext& context,
    const op_category::SequentialReadWrite& category)
{
    auto opIter = findNextOp(opType, category);
    if ((opIter == opQueue_.end()) ||
        (opIter->op_ != opType)) {
        return;
    }

    startNextOp(opType, opIter->id_, opIter->length_, context);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
template <typename TContext>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::startNextOp(
    OpType opType,
    const TContext& context,
    const op_category::ParallelReadWrite& category)
{
    auto opIter = findNextOp(opType, category);
    if (opIter == opQueue_.end()) {
        return;
    }

    startNextOp(opType, opIter->id_, opIter->length_, context);
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
template <typename TContext>
void DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::startNextOp(
    OpType opType,
    DeviceIdType id,
    std::size_t length,
    const TContext& context)
{
    auto devIter = findDeviceInfo(id);
    GASSERT(devIter != infos_.end());

    if (opType == OpType::Read) {
        GASSERT(devIter->canReadHandler_);
        GASSERT(devIter->readCompleteHandler_);
        currReadDevInfo_ = devIter;
        device_.startRead(id, length, context);
    }
    else {
        GASSERT(opType == OpType::Write);
        GASSERT(devIter->canWriteHandler_);
        GASSERT(devIter->writeCompleteHandler_);
        currWriteDevInfo_ = devIter;
        device_.startWrite(id, length, context);
    }
}


template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
typename DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::OpQueueIterator
DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::findNextOp(
    OpType opType,
    const op_category::SequentialReadWrite& category)
{
    static_cast<void>(opType);
    static_cast<void>(category);
    return opQueue_.begin();
}

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
typename DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::OpQueueIterator
DeviceOpQueue<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>::findNextOp(
    OpType opType,
    const op_category::ParallelReadWrite& category)
{
    static_cast<void>(category);
    return std::find_if(opQueue_.begin(), opQueue_.end(),
        [opType](const OpInfo& elem) -> bool
        {
            return (elem.op_ == opType);
        });
}

template <typename TDevice>
DeviceOpQueue<TDevice, 1>::DeviceOpQueue(
    Device& device)
    : device_(device)
{
}

template <typename TDevice>
typename DeviceOpQueue<TDevice, 1>::Device&
DeviceOpQueue<TDevice, 1>::device()
{
    return device_;
}

template <typename TDevice>
const typename DeviceOpQueue<TDevice, 1>::Device&
DeviceOpQueue<TDevice, 1>::device() const
{
    return device_;
}

template <typename TDevice>
template <typename TFunc>
void DeviceOpQueue<TDevice, 1>::setCanReadHandler(
    DeviceIdType id,
    TFunc&& func)
{
    static_cast<void>(id);
    device_.setCanReadHandler(std::forward<TFunc>(func));
}

template <typename TDevice>
template <typename TFunc>
void DeviceOpQueue<TDevice, 1>::setCanWriteHandler(
    DeviceIdType id,
    TFunc&& func)
{
    static_cast<void>(id);
    device_.setCanWriteHandler(std::forward<TFunc>(func));
}

template <typename TDevice>
template <typename TFunc>
void DeviceOpQueue<TDevice, 1>::setReadCompleteHandler(
    DeviceIdType id,
    TFunc&& func)
{
    static_cast<void>(id);
    device_.setReadCompleteHandler(std::forward<TFunc>(func));
}

template <typename TDevice>
template <typename TFunc>
void DeviceOpQueue<TDevice, 1>::setWriteCompleteHandler(
    DeviceIdType id,
    TFunc&& func)
{
    static_cast<void>(id);
    device_.setWriteCompleteHandler(std::forward<TFunc>(func));
}

template <typename TDevice>
template <typename... TArgs>
void DeviceOpQueue<TDevice, 1>::startRead(
    TArgs&&... args)
{
    device_.startRead(std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
bool DeviceOpQueue<TDevice, 1>::cancelRead(
    DeviceIdType id,
    TArgs&&... args)
{
    static_cast<void>(id);
    return device_.cancelRead(std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
void DeviceOpQueue<TDevice, 1>::startWrite(
    TArgs&&... args)
{
    device_.startWrite(std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
bool DeviceOpQueue<TDevice, 1>::cancelWrite(
    DeviceIdType id,
    TArgs&&... args)
{
    static_cast<void>(id);
    return device_.cancelWrite(std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
bool DeviceOpQueue<TDevice, 1>::canRead(
    TArgs&&... args)
{
    return device_.canRead(std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
bool DeviceOpQueue<TDevice, 1>::canWrite(
    TArgs&&... args)
{
    return device_.canWrite(std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
typename DeviceOpQueue<TDevice, 1>::CharType
DeviceOpQueue<TDevice, 1>::read(
    TArgs&&... args)
{
    return device_.read(std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
void DeviceOpQueue<TDevice, 1>::write(
    TArgs&&... args)
{
    device_.write(std::forward<TArgs>(args)...);
}

}  // namespace device

}  // namespace embxx

