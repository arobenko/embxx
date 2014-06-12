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

namespace details
{

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
class DeviceOpQueueBase
{
public:
    typedef TDevice Device;
    static const std::size_t Size = TSize;
    typedef TCanDoOpHandler CanDoOpHandler;
    typedef TOpCompleteHandler OpCompleteHandler;
    typedef typename Device::CharType CharType;
    typedef typename Device::DeviceIdType DeviceIdType;

    explicit DeviceOpQueueBase(Device& device)
      : device_(device)
    {
    }

    ~DeviceOpQueueBase() = default;

    Device& device()
    {
        return device_;
    }

    const Device& device() const
    {
        return device_;
    }

    template <typename TFunc>
    void setCanReadHandler(DeviceIdType id, TFunc&& func)
    {
        auto iter = findDeviceInfo(id);
        if (iter == infos_.end()) {
            GASSERT(!"Too many devices");
            return;
        }
        iter->canReadHandler_ = std::forward<TFunc>(func);
    }

    template <typename TFunc>
    void setCanWriteHandler(DeviceIdType id, TFunc&& func)
    {
        auto iter = findDeviceInfo(id);
        if (iter == infos_.end()) {
            GASSERT(!"Too many devices");
            return;
        }
        iter->canWriteHandler_ = std::forward<TFunc>(func);
    }

    template <typename TFunc>
    void setReadCompleteHandler(DeviceIdType id, TFunc&& func)
    {
        auto iter = findDeviceInfo(id);
        if (iter == infos_.end()) {
            GASSERT(!"Too many devices");
            return;
        }
        iter->readCompleteHandler_ = std::forward<TFunc>(func);
    }

    template <typename TFunc>
    void setWriteCompleteHandler(DeviceIdType id, TFunc&& func)
    {
        auto iter = findDeviceInfo(id);
        if (iter == infos_.end()) {
            GASSERT(!"Too many devices");
            return;
        }
        iter->writeCompleteHandler_ = std::forward<TFunc>(func);
    }

protected:
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

    DeviceInfoIter findDeviceInfo(DeviceIdType id)
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

    Device& device_;
    DeviceInfosArray infos_;
};

template <typename TDevice,
          std::size_t TSize,
          typename TOpCategory,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
class DeviceOpQueueImpl;

template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler,
          typename TOpCompleteHandler>
class DeviceOpQueueImpl<
    TDevice,
    TSize,
    embxx::device::op_category::SequentialReadWrite,
    TCanDoOpHandler,
    TOpCompleteHandler> : public DeviceOpQueueBase<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler>
{
    typedef DeviceOpQueueBase<TDevice, TSize, TCanDoOpHandler, TOpCompleteHandler> Base;
public:
    typedef typename Base::Device Device;
    static const std::size_t Size = Base::Size;
    typedef typename Base::CharType CharType;
    typedef typename Base::DeviceIdType DeviceIdType;

    explicit DeviceOpQueueImpl(Device& device)
      : Base(device),
        currDevInfo_(Base::infos_.end()),
        currOp_(OpType::Invalid)
    {
        Base::device_.setCanReadHandler(
            std::bind(&DeviceOpQueueImpl::canDoOpHandler, this, OpType::Read));
        Base::device_.setCanWriteHandler(
            std::bind(&DeviceOpQueueImpl::canDoOpHandler, this, OpType::Write));
        Base::device_.setReadCompleteHandler(
            std::bind(
                &DeviceOpQueueImpl::opCompleteHandler,
                this,
                std::placeholders::_1,
                OpType::Read));
        Base::device_.setWriteCompleteHandler(
            std::bind(
                &DeviceOpQueueImpl::opCompleteHandler,
                this,
                std::placeholders::_1,
                OpType::Write));

    }

    ~DeviceOpQueueImpl()
    {
        Base::device_.setCanReadHandler(nullptr);
        Base::device_.setCanWriteHandler(nullptr);
        Base::device_.setReadCompleteHandler(nullptr);
        Base::device_.setWriteCompleteHandler(nullptr);
    }

    void startRead(
        DeviceIdType id,
        std::size_t length,
        context::EventLoop)
    {
        startNewOpReq(id, length, OpType::Read);
    }

    bool cancelRead(
        DeviceIdType id,
        context::EventLoop)
    {
        return cancelOpReq(id, OpType::Read);
    }

    bool cancelRead(
        DeviceIdType id,
        context::Interrupt context)
    {
        static_cast<void>(id);
        GASSERT(currDevInfo_ != Base::infos_.end());
        GASSERT(currDevInfo_->id_ == id);
        auto cancelResult = Base::device_.cancelRead(context);
        GASSERT(cancelResult);
        startNextOp(context);
        return cancelResult;
    }

    void startWrite(
        DeviceIdType id,
        std::size_t length,
        context::EventLoop)
    {
        return startNewOpReq(id, length, OpType::Write);
    }

    bool cancelWrite(DeviceIdType id, context::EventLoop)
    {
        return cancelOpReq(id, OpType::Write);
    }

    bool canRead(DeviceIdType id, context::Interrupt context)
    {
        GASSERT(opQueue_.front().id_ == id);
        static_cast<void>(id);
        return Base::device_.canRead(context);
    }

    bool canWrite(DeviceIdType id, context::Interrupt context)
    {
        GASSERT(opQueue_.front().id_ == id);
        static_cast<void>(id);
        return Base::device_.canWrite(context);
    }

    CharType read(DeviceIdType id, context::Interrupt context)
    {
        GASSERT(opQueue_.front().id_ == id);
        static_cast<void>(id);
        return Base::device_.read(context);
    }

    void write(DeviceIdType id, CharType value, context::Interrupt context)
    {
        GASSERT(opQueue_.front().id_ == id);
        static_cast<void>(id);
        Base::device_.write(value, context);
    }

private:
    typedef typename Base::OpType OpType;

    struct OpInfo
    {
        OpInfo() : id_(0), length_(0), op_(OpType::Invalid) {}
        OpInfo(DeviceIdType addr, std::size_t length, OpType op)
            : id_(addr), length_(length), op_(op) {}

        DeviceIdType id_;
        std::size_t length_;
        OpType op_;
    };

    typedef embxx::container::StaticQueue<OpInfo, Size> OpQueue;
    typedef typename OpQueue::iterator OpQueueIterator;

    typedef typename Base::DeviceInfoIter DeviceInfoIter;

    typedef embxx::device::context::EventLoop EventLoopContext;
    typedef embxx::device::context::Interrupt InterruptContext;

    void startNewOpReq(
        DeviceIdType id,
        std::size_t length,
        OpType op)
    {
        auto suspResult = Base::device_.suspend(EventLoopContext());

        // Suspend is successful if and only if the queue is empty
        GASSERT(suspResult == (!opQueue_.empty()));

        GASSERT(findOpInfo(id) == opQueue_.end());

        OpInfo info(id, length, op);
        opQueue_.push_back(std::move(info));
        if (suspResult) {
            Base::device_.resume(EventLoopContext());
            return;
        }

        startNextOp(EventLoopContext());
    }

    bool cancelOpReq(
        DeviceIdType id,
        OpType op)
    {
        auto suspResult = Base::device_.suspend(EventLoopContext());

        // Suspend is successful if and only if the queue is empty
        GASSERT(suspResult == (!opQueue_.empty()));

        auto guard =
            embxx::util::makeScopeGuard(
                [this, suspResult]()
                {
                    if (suspResult) {
                        Base::device().resume(EventLoopContext());
                    }
                });

        auto iter = findOpInfo(id);
        if ((iter == opQueue_.end()) ||
            (iter->op_ != op)) {
            return false; // resuming operation
        }

        GASSERT(currDevInfo_ != Base::infos_.end());
        if (id != currDevInfo_->id_) {
            GASSERT(opQueue_.front().id_ != id);
            opQueue_.erase(iter);
            return true; // resuming operation
        }

        // Cancelling current op
        guard.release(); // No resume on return

        GASSERT(opQueue_.front().id_ == id);
        bool result = false;
        if (op == OpType::Read) {
            result = Base::device_.cancelRead(EventLoopContext());
        }
        else {
            GASSERT(op == OpType::Write);
            result = Base::device_.cancelWrite(EventLoopContext());
        }

        GASSERT(result); // cancel must succeed
        static_cast<void>(result);
        opQueue_.popFront();
        startNextOp(EventLoopContext());
        return result;
    }

    OpQueueIterator findOpInfo(DeviceIdType id)
    {
        return std::find_if(opQueue_.begin(), opQueue_.end(),
            [id](const OpInfo& elem) -> bool
            {
                return elem.id_ == id;
            });
    }

    template <typename TContext>
    void startNextOp(TContext context)
    {
        if (opQueue_.empty()) {
            return;
        }

        auto& opInfo = opQueue_.front();
        currDevInfo_ = Base::findDeviceInfo(opInfo.id_);
        GASSERT(currDevInfo_ != Base::infos_.end());

        if (opInfo.op_ == OpType::Read) {
            Base::device_.startRead(opInfo.id_, opInfo.length_, context);
        }
        else {
            GASSERT(opInfo.op_ == OpType::Write);
            Base::device_.startWrite(opInfo.id_, opInfo.length_, context);
        }
    }

    void canDoOpHandler(OpType op)
    {
        GASSERT(currDevInfo_ != Base::infos_.end());
        if (op == OpType::Read) {
            GASSERT(currDevInfo_->canReadHandler_);
            currDevInfo_->canReadHandler_();
        }
        else {
            GASSERT(op == OpType::Write);
            GASSERT(currDevInfo_->canWriteHandler_);
            currDevInfo_->canWriteHandler_();
        }
    }

    void opCompleteHandler(const embxx::error::ErrorStatus& err, OpType op)
    {
        GASSERT(currDevInfo_ != Base::infos_.end());
        GASSERT(!opQueue_.empty());
        GASSERT(currDevInfo_->id_ == opQueue_.front().id_);
        GASSERT(opQueue_.front().op_ == op);
        opQueue_.popFront();

        auto devInfo = currDevInfo_;
        currDevInfo_ = Base::infos_.end();
        startNextOp(context::Interrupt());

        if (op == OpType::Read) {
            GASSERT(devInfo->readCompleteHandler_);
            devInfo->readCompleteHandler_(err);
        }
        else {
            GASSERT(op == OpType::Write);
            devInfo->writeCompleteHandler_(err);
        }
    }

private:
    DeviceInfoIter currDevInfo_;
    OpQueue opQueue_;
    OpType currOp_;
};

}  // namespace details

/// @ingroup device
/// @brief Device operations queue.
/// @details Some devices may be used to communicate to various independent
///          entities using some kind of identification, such as address
///          in I2C or chip select GPIO line in SPI protocols. This class
///          provides management of independent read/write operations to
///          independent entities and forwards the operation requests to the
///          underlying device using FIFO policy. This class also has a template
///          specialisation when TSize parameter is 1. It forwards all the
///          requests to the device control object without any queue management
///          overhead.
/// @tparam TDevice Actual device (peripheral) control object. It must provide
///         the following interface:
///         @code
///         // Definition of ID type
///         typedef ... DeviceIdType;
///
///         // Definition of device operation category, may be either
///         // embxx::device::op_category::SequentialReadWrite (the read and
///         // write must be sequential, i.e. cannot be executed in parallel -
///         // I2C interface) or  embxx::device::op_category::ParallelReadWrite
///         // (the read and write operations are independent and may be
///         // executed in parallel). This definition is used to identify
///         // whether read and write operations to the same device can be executed
///         // at the same time or not.
///         typedef ... OpCategory;
///
///         // Definition of single character type
///         typedef ... CharType;
///
///         // Set the "can read" interrupt callback which has "void ()"
///         // signature. The callback must be called when there is at least
///         // one byte available. The callback will perform multiple canRead()
///         // and read() calls until canRead() returns false. This function is
///         // called in NON-interrupt (event loop) context when the driver
///         // object is constructed, but the callback itself is expected to
///         // be called by the device control object in interrupt context.
///         template <typename TFunc>
///         void setCanReadHandler(TFunc&& func);
///
///         // Set the "can write" interrupt callback which has "void ()"
///         // signature. The callback must be called when there is a space for
///         // at least one byte to be written. The callback will perform multiple
///         // canWrite() and write() calls until canWrite() returns false. This
///         // function is called in NON-interrupt (event loop) context when
///         // the driver object is constructed, but the callback itself is
///         // expected to be called by the device control object in interrupt
///         // context.
///         template <typename TFunc>
///         void setCanWriteHandler(TFunc&& func);
///
///         // Set the "read complete" interrupt callback which has
///         // "void (const embxx::error::ErrorStatus&)" signature. The callback
///         // must be called when read operation is complete (either successfully
///         // or with errors) and read interrupts are disabled, i.e. no more
///         // "can read" callback calls will follow until next call to startRead().
///         // The error status passed as a parameter must indicate the status
///         // of the read operation. This function is called in NON-interrupt
///         // (event loop) context, but the callback itself is expected
///         // to be called by the device control object in interrupt context.
///         template <typename TFunc>
///         void setReadCompleteHandler(TFunc&& func);
///
///         // Set the "write complete" interrupt callback which has
///         // "void (const embxx::error::ErrorStatus&)" signature. The callback
///         // must be called when write operation is complete (either successfully
///         // or with errors) and write interrupts are disabled, i.e. no more
///         // "can write" callback calls will follow until next call to startWrite().
///         // The error status passed as a parameter must indicate the status
///         // of the write operation. This function is called in NON-interrupt
///         // (event loop) context, but the callback itself is expected
///         // to be called by the device control object in interrupt context.
///         template <typename TFunc>
///         void setWriteCompleteHandler(TFunc&& func);
///
///         // Start read operation. The device will perform some configuration
///         // if needed and enable read interrupts, i.e. interrupt when there
///         // is at least one character to read. The "context" is a dummy
///         // parameter that indicates whether the function is executed in
///         // NON-interrupt (event loop) or interrupt contexts.
///         void startRead(DeviceIdType id, std::size_t length, embxx::device::context::EventLoop context);
///         void startRead(DeviceIdType id, std::size_t length, embxx::device::context::Interrupt context);
///
///         // Cancel current read operation. The return value indicates whether the
///         // read operation was cancelled. The "context" is a dummy
///         // parameter that indicates whether the function is executed in
///         // NON-interrupt (event loop) or interrupt contexts.
///         bool cancelRead(embxx::device::context::EventLoop context);
///         bool cancelRead(embxx::device::context::Interrupt context)
///
///         // Start write operation. The device will perform some configuration
///         // if needed and enable write interrupts, i.e. interrupt when there
///         // is space for at least one character to write. The "context" is a
///         // dummy parameter that indicates whether the function is executed
///         // in NON-interrupt (event loop) or interrupt contexts.
///         void startWrite(DeviceIdType id, std::size_t length, embxx::device::context::EventLoop context);
///         void startWrite(DeviceIdType id, std::size_t length, embxx::device::context::Interrupt context);
///
///         // Cancel current write operation. The return value indicates whether
///         // the write operation was cancelled. The "context" is a dummy
///         // parameter that indicates whether the function is executed in
///         // NON-interrupt (event loop) or interrupt contexts.
///         bool cancelWrite(embxx::device::context::EventLoop context);
///         bool cancelWrite(embxx::device::context::Interrupt context)
///
///         // Suspend current read/write operation(s). The "context" is a dummy
///         // parameter that indicates whether the function is executed in
///         // NON-interrupt (event loop) or interrupt contexts. This function
///         // can be called only in event loop context.
///         bool suspend(embxx::device::context::EventLoop context);
///
///         // Resume suspended read/write operations. The "context" is a dummy
///         // parameter that indicates whether the function is executed in
///         // NON-interrupt (event loop) or interrupt contexts. This function
///         // can be called only in event loop context.
///         void resume(embxx::device::context::EventLoop context);
///
///         // Inquiry whether there is at least one character to be
///         // read. Will be called in the interrupt context. May be called
///         // multiple times in the same interrupt.
///         bool canRead(embxx::device::context::Interrupt context);
///
///         // Inquiry whether there is a space for at least one character to
///         // be written. Will be called in the interrupt context. May be called
///         // multiple times in the same interrupt.
///         bool canWrite(embxx::device::context::Interrupt context);
///
///         // Read one character. Precondition to this call: canRead() returns
///         // true. Will be called in the interrupt context. May be called
///         // multiple times in the same interrupt.
///         CharType read(embxx::device::context::Interrupt context);
///
///         // Write one character. Precondition to this call: canWrite() returns
///         // true. Will be called in the interrupt context. May be called
///         // multiple times in the same interrupt.
///         void write(CharType value, embxx::device::context::Interrupt context);
///         @endcode
/// @tparam TSize Size of the queue - number of independent entities operations
///         to which this class needs to manage.
/// @tparam TCanDoOpHandler A function class that is supposed to store "can read"
///         or "can write" callbacks from the driver. Must be either std::function
///         or embxx::util::StaticFunction and provide "void()" calling interface.
/// @tparam TOpCompleteHandler A function class that is supposed to store
///         "read complete" or "write complete" callbacks from the driver.
///         Must be either std::function or embxx::util::StaticFunction and
///         provide "void (const embxx::error::ErrorStatus&)" calling interface.
/// @headerfile embxx/device/DeviceOpQueue.h
template <typename TDevice,
          std::size_t TSize,
          typename TCanDoOpHandler = embxx::util::StaticFunction<void()>,
          typename TOpCompleteHandler = embxx::util::StaticFunction<void (const embxx::error::ErrorStatus&)> >
class DeviceOpQueue : public details::DeviceOpQueueImpl<TDevice, TSize, typename TDevice::OpCategory, TCanDoOpHandler, TOpCompleteHandler>
{
    typedef details::DeviceOpQueueImpl<
            TDevice,
            TSize,
            typename TDevice::OpCategory,
            TCanDoOpHandler,
            TOpCompleteHandler
        > Base;
public:
    /// @brief Type of the peripheral control object.
    typedef TDevice Device;

    /// @brief Size of the queue
    static const std::size_t Size = TSize;

    /// @brief Type of "can read" and "can write" callback holder class.
    typedef TCanDoOpHandler CanDoOpHandler;

    /// @brief Type of "read complete" and "write complete" callback holder class.
    typedef TOpCompleteHandler OpCompleteHandler;

    /// @brief Definition of single character type.
    typedef typename Device::CharType CharType;

    /// @brief Definition of device id type.
    typedef typename Device::DeviceIdType DeviceIdType;

    /// @brief Definition of operations category.
    typedef typename Device::OpCategory OpCategory;

    /// @brief Constructor
    /// @param device Reference to device (peripheral) control object
    explicit DeviceOpQueue(Device& device)
      : Base(device)
    {
    }

    /// @brief Copy constructor is default.
    DeviceOpQueue(const DeviceOpQueue&) = default;

    /// @brief Move constructor is default.
    DeviceOpQueue(DeviceOpQueue&&) = default;

    /// @brief Destructor
    ~DeviceOpQueue() = default;

    /// @brief Copy assignment is deleted
    DeviceOpQueue& operator=(const DeviceOpQueue&) = delete;

    /// @brief Move assignment is deleted
    DeviceOpQueue& operator=(DeviceOpQueue&&) = delete;

    /// @brief Get reference to device (peripheral) control object.
    Device& device()
    {
        return Base::device();
    }

    /// @brief Const version of device().
    const Device& device() const
    {
        return Base::device();
    }

    /// @brief Set the "can read" callback.
    /// @details The callback will be called when read operation to the
    ///          device with provided id is scheduled and there is "can read"
    ///          interrupt. The callback is called in interrupt context.
    /// @param[in] id ID of the entity to which read operation will be performed.
    /// @param[in] func Callable functor with signature "void ()".
    template <typename TFunc>
    void setCanReadHandler(DeviceIdType id, TFunc&& func)
    {
        Base::setCanReadHandler(id, std::forward<TFunc>(func));
    }

    /// @brief Set the "can write" callback.
    /// @details The callback will be called when write operation to the
    ///          device with provided id is scheduled and there is "can write"
    ///          interrupt. The callback is called in interrupt context.
    /// @param[in] id ID of the entity to which write operation will be performed.
    /// @param[in] func Callable functor with signature "void ()".
    template <typename TFunc>
    void setCanWriteHandler(DeviceIdType id, TFunc&& func)
    {
        Base::setCanWriteHandler(id, std::forward<TFunc>(func));
    }

    /// @brief Set the "read complete" callback.
    /// @details The callback will be called when read operation to the
    ///          device with provided id is complete (whether successfully or
    ///          when error occurred). The callback is called in interrupt context.
    /// @param[in] id ID of the entity to which read operation will be performed.
    /// @param[in] func Callable functor with signature "void (const embxx::error::ErrorStatus&)".
    template <typename TFunc>
    void setReadCompleteHandler(DeviceIdType id, TFunc&& func)
    {
        Base::setReadCompleteHandler(id, std::forward<TFunc>(func));
    }

    /// @brief Set the "write complete" callback.
    /// @details The callback will be called when write operation to the
    ///          device with provided id is complete (whether successfully or
    ///          when error occurred). The callback is called in interrupt context.
    /// @param[in] id ID of the entity to which write operation will be performed.
    /// @param[in] func Callable functor with signature "void (const embxx::error::ErrorStatus&)".
    template <typename TFunc>
    void setWriteCompleteHandler(DeviceIdType id, TFunc&& func)
    {
        Base::setWriteCompleteHandler(id, std::forward<TFunc>(func));
    }

    /// @brief Start read operation in event loop context.
    /// @param[in] id ID of the entity to which read should be performed.
    /// @param[in] length Number of bytes about to be read.
    /// @param[in] context Tag parameter - indication that function is called
    ///            in event loop context
    void startRead(
        DeviceIdType id,
        std::size_t length,
        context::EventLoop context)
    {
        Base::startRead(id, length, context);
    }

    /// @brief Cancel read operation in either event loop or interrupt context.
    /// @param[in] id ID of the entity to which read operation should be cancelled.
    /// @param[in] context Tag parameter - indication of call context. May be
    ///            either embxx::device::context::EventLoop or
    ///            embxx::device::context::Interrupt
    /// @return true in case the read was successfully cancelled.
    template <typename TContext>
    bool cancelRead(
        DeviceIdType id,
        TContext context)
    {
        return Base::cancelRead(id, context);
    }

    /// @brief Start write operation in event loop context.
    /// @param[in] id ID of the entity to which write should be performed.
    /// @param[in] length Number of bytes about to be written.
    /// @param[in] context Tag parameter - indication that function is called
    ///            in event loop context
    void startWrite(
        DeviceIdType id,
        std::size_t length,
        context::EventLoop context)
    {
        Base::startWrite(id, length, context);
    }

    /// @brief Cancel write operation in event loop context.
    /// @param[in] id ID of the entity to which write operation should be cancelled.
    /// @param[in] context Tag parameter - indication that function is called
    ///            in event loop context
    /// @return true in case the write was successfully cancelled.
    bool cancelWrite(
        DeviceIdType id,
        context::EventLoop context)
    {
        return Base::cancelWrite(id, context);
    }

    /// @brief Suspend previous issued read/write operations
    /// @param[in] id ID of the entity to which operations should be suspended.
    /// @param[in] context Tag parameter - indication that function is called
    ///            in event loop context
    /// @return true in case the operations were successfully suspended.
    bool suspend(DeviceIdType id, context::EventLoop context)
    {
        GASSERT(!"Currently suspend/resume operations are not supported.");
        return false;
    }

    /// @brief Resumed previously suspended operations.
    /// @param[in] id ID of the entity to which operations should be resumed.
    /// @param[in] context Tag parameter - indication that function is called
    ///            in event loop context
    /// @return true in case the operations were successfully suspended.
    void resume(DeviceIdType id, context::EventLoop context)
    {
        GASSERT(!"Currently suspend/resume operations are not supported.");
    }

    /// @brief Inquiry whether the device has received character that can be read.
    /// @param[in] id ID of the entity to which request is performed
    /// @param[in] context Tag parameter - indication that function is called
    ///            in interrupt context.
    /// @return true in case a character may be read, false otherwise.
    bool canRead(DeviceIdType id, context::Interrupt context)
    {
        return Base::canRead(id, context);
    }

    /// @brief Inquiry whether the device has place for one more character in
    ///        its outgoing FIFO.
    /// @param[in] id ID of the entity to which request is performed.
    /// @param[in] context Tag parameter - indication that function is called
    ///            in interrupt context.
    /// @return true in case a character may be written, false otherwise.
    bool canWrite(DeviceIdType id, context::Interrupt context)
    {
        return Base::canWrite(id, context);
    }

    /// @brief Read character from incoming FIFO of the device.
    /// @param[in] id ID of the entity to which request is performed.
    /// @param[in] context Dummy parameter - indication that function is called
    ///            in interrupt context.
    /// @return Read character.
    /// @pre @code canRead() == true @endcode
    CharType read(DeviceIdType id, context::Interrupt context)
    {
        return Base::read(id, context);
    }

    /// @brief Write character to outgoing FIFO of the device.
    /// @param[in] id ID of the entity to which request is performed.
    /// @param value Character to be written.
    /// @param[in] context Dummy parameter - indication that function is called
    ///            in interrupt context.
    /// @pre @code canWrite() == true @endcode
    void write(DeviceIdType id, CharType value, context::Interrupt context)
    {
        Base::write(id, value, context);
    }
};

/// @cond DOCUMENT_DEVICE_OP_QUEUE_SPECIALISATION
template <typename TDevice>
class DeviceOpQueue<TDevice, 1>
{
public:
    typedef TDevice Device;
    static const std::size_t Size = 1;

    typedef typename Device::CharType CharType;
    typedef typename Device::DeviceIdType DeviceIdType;
    typedef typename Device::OpCategory OpCategory;

    explicit DeviceOpQueue(Device& device)
      : device_(device)
    {
    }

    DeviceOpQueue(const DeviceOpQueue&) = default;
    ~DeviceOpQueue() = default;

    DeviceOpQueue& operator=(const DeviceOpQueue&) = delete;

    Device& device()
    {
        return device_;
    }

    const Device& device() const
    {
        return device_;
    }

    template <typename TFunc>
    void setCanReadHandler(DeviceIdType id, TFunc&& func)
    {
        static_cast<void>(id);
        device_.setCanReadHandler(std::forward<TFunc>(func));
    }

    template <typename TFunc>
    void setCanWriteHandler(DeviceIdType id, TFunc&& func)
    {
        static_cast<void>(id);
        device_.setCanWriteHandler(std::forward<TFunc>(func));
    }

    template <typename TFunc>
    void setReadCompleteHandler(DeviceIdType id, TFunc&& func)
    {
        static_cast<void>(id);
        device_.setReadCompleteHandler(std::forward<TFunc>(func));
    }

    template <typename TFunc>
    void setWriteCompleteHandler(DeviceIdType id, TFunc&& func)
    {
        static_cast<void>(id);
        device_.setWriteCompleteHandler(std::forward<TFunc>(func));
    }

    template <typename... TArgs>
    void startRead(TArgs&&... args)
    {
        device_.startRead(std::forward<TArgs>(args)...);
    }

    template <typename... TArgs>
    bool cancelRead(DeviceIdType id, TArgs&&... args)
    {
        static_cast<void>(id);
        return device_.cancelRead(std::forward<TArgs>(args)...);
    }

    template <typename... TArgs>
    void startWrite(TArgs&&... args)
    {
        device_.startWrite(std::forward<TArgs>(args)...);
    }

    template <typename... TArgs>
    bool cancelWrite(DeviceIdType id, TArgs&&... args)
    {
        static_cast<void>(id);
        return device_.cancelWrite(std::forward<TArgs>(args)...);
    }

    template <typename... TArgs>
    bool suspend(DeviceIdType id, TArgs&&... args)
    {
        static_cast<void>(id);
        return device_.suspend(std::forward<TArgs>(args)...);
    }

    template <typename... TArgs>
    void resume(DeviceIdType id, TArgs&&... args)
    {
        static_cast<void>(id);
        device_.resume(std::forward<TArgs>(args)...);
    }

    template <typename... TArgs>
    bool canRead(DeviceIdType id, TArgs&&... args)
    {
        static_cast<void>(id);
        return device_.canRead(std::forward<TArgs>(args)...);
    }

    template <typename... TArgs>
    bool canWrite(DeviceIdType id, TArgs&&... args)
    {
        static_cast<void>(id);
        return device_.canWrite(std::forward<TArgs>(args)...);
    }

    template <typename... TArgs>
    CharType read(DeviceIdType id, TArgs&&... args)
    {
        static_cast<void>(id);
        return device_.read(std::forward<TArgs>(args)...);
    }

    template <typename... TArgs>
    void write(DeviceIdType id, TArgs&&... args)
    {
        static_cast<void>(id);
        device_.write(std::forward<TArgs>(args)...);
    }

private:
    Device& device_;
};
/// @endcond

}  // namespace device

}  // namespace embxx

