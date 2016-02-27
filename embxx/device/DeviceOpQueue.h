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

namespace embxx
{

namespace device
{

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
///         // is at least one character to read. The "context" is a tag
///         // parameter that indicates whether the function is executed in
///         // NON-interrupt (event loop) or interrupt contexts.
///         // In case the device supports execution of read and write at the
///         // same time, the call to startRead() may occur when the device
///         // is in the "suspended" state, i.e. between calls to suspend()
///         // and resume().
///         void startRead(DeviceIdType id, std::size_t length, embxx::device::context::EventLoop context);
///         void startRead(DeviceIdType id, std::size_t length, embxx::device::context::Interrupt context);
///
///         // Cancel current read operation. The return value indicates whether the
///         // read operation was cancelled. The "context" is a tag
///         // parameter that indicates whether the function is executed in
///         // NON-interrupt (event loop) or interrupt contexts.
///         bool cancelRead(embxx::device::context::EventLoop context);
///         bool cancelRead(embxx::device::context::Interrupt context)
///
///         // Start write operation. The device will perform some configuration
///         // if needed and enable write interrupts, i.e. interrupt when there
///         // is space for at least one character to write. The "context" is a
///         // tag parameter that indicates whether the function is executed
///         // in NON-interrupt (event loop) or interrupt contexts.
///         // In case the device supports execution of read and write at the
///         // same time, the call to startWrite() may occur when the device
///         // is in the "suspended" state, i.e. between calls to suspend()
///         // and resume().
///         void startWrite(DeviceIdType id, std::size_t length, embxx::device::context::EventLoop context);
///         void startWrite(DeviceIdType id, std::size_t length, embxx::device::context::Interrupt context);
///
///         // Cancel current write operation. The return value indicates whether
///         // the write operation was cancelled. The "context" is a tag
///         // parameter that indicates whether the function is executed in
///         // NON-interrupt (event loop) or interrupt contexts.
///         bool cancelWrite(embxx::device::context::EventLoop context);
///         bool cancelWrite(embxx::device::context::Interrupt context)
///
///         // Suspend current read/write operation(s). The "context" is a tag
///         // parameter that indicates whether the function is executed in
///         // NON-interrupt (event loop) or interrupt contexts. This function
///         // can be called only in event loop context.
///         bool suspend(embxx::device::context::EventLoop context);
///
///         // Resume suspended read/write operations. The "context" is a tag
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
class DeviceOpQueue
{
    enum class OpType {
        Invalid,
        Read,
        Write
    };

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

    /// @brief Constructor
    /// @param dev Reference to device (peripheral) control object
    explicit DeviceOpQueue(Device& dev)
      : device_(dev),
        suspended_(false)
    {
        device_.setCanReadHandler(
            [this]()
            {
                canDoInterruptHandler(OpType::Read);
            });

        device_.setReadCompleteHandler(
            [this](const embxx::error::ErrorStatus& es)
            {
                opCompleteInterruptHandler(es, OpType::Read);
            });

        device_.setCanWriteHandler(
            [this]()
            {
                canDoInterruptHandler(OpType::Write);
            });

        device_.setWriteCompleteHandler(
            [this](const embxx::error::ErrorStatus& es)
            {
                opCompleteInterruptHandler(es, OpType::Write);
            });
    }

    /// @brief Copy constructor is deleted.
    DeviceOpQueue(const DeviceOpQueue&) = delete;

    /// @brief Move constructor is deleted.
    DeviceOpQueue(DeviceOpQueue&&) = delete;

    /// @brief Destructor
    ~DeviceOpQueue()
    {
        device_.cancelRead(EventLoopContext());
        device_.cancelWrite(EventLoopContext());
        device_.setCanReadHandler(nullptr);
        device_.setReadCompleteHandler(nullptr);
        device_.setCanWriteHandler(nullptr);
        device_.setWriteCompleteHandler(nullptr);
    }

    /// @brief Copy assignment is deleted
    DeviceOpQueue& operator=(const DeviceOpQueue&) = delete;

    /// @brief Move assignment is deleted
    DeviceOpQueue& operator=(DeviceOpQueue&&) = delete;

    /// @brief Get reference to device (peripheral) control object.
    Device& device()
    {
        return device_;
    }

    /// @brief Const version of device().
    const Device& device() const
    {
        return device_;
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
        auto iter = findDeviceInfo(id);
        if (iter == infos_.end()) {
            GASSERT(!"Too many devices");
            return;
        }
        iter->canReadHandler_ = std::forward<TFunc>(func);
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
        auto iter = findDeviceInfo(id);
        if (iter == infos_.end()) {
            GASSERT(!"Too many devices");
            return;
        }
        iter->canWriteHandler_ = std::forward<TFunc>(func);
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
        auto iter = findDeviceInfo(id);
        if (iter == infos_.end()) {
            GASSERT(!"Too many devices");
            return;
        }
        iter->readCompleteHandler_ = std::forward<TFunc>(func);
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
        auto iter = findDeviceInfo(id);
        if (iter == infos_.end()) {
            GASSERT(!"Too many devices");
            return;
        }
        iter->writeCompleteHandler_ = std::forward<TFunc>(func);
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
        static_cast<void>(context);
        startNewOpReq(id, length, OpType::Read);
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
        static_cast<void>(context);
        return cancelExistingOpReq(id, OpType::Read);
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
        static_cast<void>(context);
        startNewOpReq(id, length, OpType::Write);
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
        static_cast<void>(context);
        return cancelExistingOpReq(id, OpType::Write);
    }

    /// @brief Suspend previous issued read/write operations
    /// @param[in] id ID of the entity to which operations should be suspended.
    /// @param[in] context Tag parameter - indication that function is called
    ///            in event loop context
    /// @return true in case the operations were successfully suspended.
    bool suspend(DeviceIdType id, context::EventLoop context)
    {
        auto suspResult = suspendDeviceEventLoopCtx();
        auto guard =
            embxx::util::makeScopeGuard(
                [this, suspResult]()
                {
                    if (suspResult) {
                        resumeDeviceEventLoopCtx();
                    }
                });

        auto iter = findOpInfo(id);
        if (iter == opQueue_.end()) {
            return false;
        }

        GASSERT(!iter->suspended_);
        iter->suspended_ = true;

        if (opQueue_.begin() != iter) {
            return true;
        }

        guard.release();
        return true;
    }

    /// @brief Resumed previously suspended operations.
    /// @param[in] id ID of the entity to which operations should be resumed.
    /// @param[in] context Tag parameter - indication that function is called
    ///            in event loop context
    /// @return true in case the operations were successfully suspended.
    void resume(DeviceIdType id, context::EventLoop context)
    {
        auto suspResult = suspendDeviceEventLoopCtx();
        auto guard =
            embxx::util::makeScopeGuard(
                [this, suspResult]()
                {
                    if (suspResult) {
                        resumeDeviceEventLoopCtx();
                    }
                });

        auto iter = findOpInfo(id);
        if (iter == opQueue_.end()) {
            GASSERT(!"Mustn't happen");
            return;
        }

        GASSERT(iter->suspended_);
        iter->suspended_ = false;
        if (opQueue_.begin() != iter) {
            return;
        }

        if (0 < iter->readLength_) {
            device_.startRead(iter->id_, iter->readLength_, EventLoopContext());
        }

        if (0 < iter->writeLength_) {
            device_.startWrite(iter->id_, iter->writeLength_, EventLoopContext());
        }
    }

    /// @brief Inquiry whether the device has received character that can be read.
    /// @param[in] id ID of the entity to which request is performed
    /// @param[in] context Tag parameter - indication that function is called
    ///            in interrupt context.
    /// @return true in case a character may be read, false otherwise.
    bool canRead(DeviceIdType id, context::Interrupt context)
    {
        GASSERT(opQueue_.front().id_ == id);
        static_cast<void>(id);
        return device_.canRead(context);
    }

    /// @brief Inquiry whether the device has place for one more character in
    ///        its outgoing FIFO.
    /// @param[in] id ID of the entity to which request is performed.
    /// @param[in] context Tag parameter - indication that function is called
    ///            in interrupt context.
    /// @return true in case a character may be written, false otherwise.
    bool canWrite(DeviceIdType id, context::Interrupt context)
    {
        GASSERT(opQueue_.front().id_ == id);
        static_cast<void>(id);
        return device_.canWrite(context);
    }

    /// @brief Read character from incoming FIFO of the device.
    /// @param[in] id ID of the entity to which request is performed.
    /// @param[in] context Dummy parameter - indication that function is called
    ///            in interrupt context.
    /// @return Read character.
    /// @pre @code canRead() == true @endcode
    CharType read(DeviceIdType id, context::Interrupt context)
    {
        GASSERT(opQueue_.front().id_ == id);
        static_cast<void>(id);
        return device_.read(context);
    }

    /// @brief Write character to outgoing FIFO of the device.
    /// @param[in] id ID of the entity to which request is performed.
    /// @param value Character to be written.
    /// @param[in] context Dummy parameter - indication that function is called
    ///            in interrupt context.
    /// @pre @code canWrite() == true @endcode
    void write(DeviceIdType id, CharType value, context::Interrupt context)
    {
        GASSERT(opQueue_.front().id_ == id);
        static_cast<void>(id);
        device_.write(value, context);
    }

private:
    struct DeviceInfo
    {
        DeviceInfo()
            : id_(DeviceIdType()),
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

    struct OpInfo
    {
        OpInfo()
          : id_(DeviceIdType()),
            readLength_(0),
            writeLength_(0),
            suspended_(false)
        {
        }

        OpInfo(DeviceIdType id)
          : id_(id),
            readLength_(0),
            writeLength_(0),
            suspended_(false)
        {
        }

        DeviceIdType id_;
        std::size_t readLength_;
        std::size_t writeLength_;
        bool suspended_;
    };

    typedef embxx::container::StaticQueue<OpInfo, Size> OpQueue;
    typedef typename OpQueue::iterator OpQueueIterator;

    typedef embxx::device::context::EventLoop EventLoopContext;
    typedef embxx::device::context::Interrupt InterruptContext;

    template <typename TContext>
    void startNextOpIfAvailable(TContext&& context) {
        if (opQueue_.empty()) {
            return;
        }

        auto& nextOp = opQueue_.front();
        GASSERT((0 < nextOp.readLength_) || (0 < nextOp.writeLength_));
        if (nextOp.suspended_) {
            return;
        }

        if (0 < nextOp.readLength_) {
            device_.startRead(nextOp.id_, nextOp.readLength_, std::forward<TContext>(context));
        }

        if (0 < nextOp.writeLength_) {
            device_.startWrite(nextOp.id_, nextOp.writeLength_, std::forward<TContext>(context));
        }
    }

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

    OpQueueIterator findOpInfo(DeviceIdType id)
    {
        return std::find_if(opQueue_.begin(), opQueue_.end(),
            [id](const OpInfo& elem) -> bool
            {
                return elem.id_ == id;
            });
    }

    void startNewOpReq(
        DeviceIdType id,
        std::size_t length,
        OpType op)
    {
        GASSERT(0 < length);
        auto suspResult = suspendDeviceEventLoopCtx();

        auto iter = findOpInfo(id);
        if (iter == opQueue_.end()) {
            opQueue_.pushBack(OpInfo(id));
            iter = findOpInfo(id);
        }

        if (op == OpType::Read) {
            GASSERT(iter->readLength_ == 0);
            iter->readLength_ = length;

            if ((opQueue_.begin() == iter) && (0 < iter->writeLength_)) {
                // Write is progress, add read
                device_.startRead(id, length, EventLoopContext());
            }
        }
        else  {
            GASSERT(op == OpType::Write);
            GASSERT(iter->writeLength_ == 0);
            iter->writeLength_ = length;

            if ((opQueue_.begin() == iter) && (0 < iter->readLength_)) {
                // Read is progress, add write
                device_.startWrite(id, length, EventLoopContext());
            }
        }

        if (suspResult) {
            resumeDeviceEventLoopCtx();
            return;
        }

        if (opQueue_.front().id_ != id) {
            return;
        }

        GASSERT(opQueue_.begin() == iter);

        if (iter->suspended_) {
            // Wait until resumed explicitly
            return;
        }

        if (op == OpType::Read) {
            device_.startRead(id, length, EventLoopContext());
        }
        else {
            GASSERT(op == OpType::Write);
            device_.startWrite(id, length, EventLoopContext());
        }
    }

    bool cancelExistingOpReq(
        DeviceIdType id,
        OpType op)
    {
        auto suspResult = suspendDeviceEventLoopCtx();
        auto guard =
            embxx::util::makeScopeGuard(
                [this, suspResult]()
                {
                    if (suspResult) {
                        resumeDeviceEventLoopCtx();
                    }
                });

        auto iter = findOpInfo(id);
        if (iter == opQueue_.end()) {
            return false;
        }

        bool currentOp = (iter == opQueue_.begin());

        if (op == OpType::Read) {
            if (iter->readLength_ == 0) {
                return false;
            }

            iter->readLength_ = 0;
            if (currentOp) {
                auto cancelResult = device_.cancelRead(EventLoopContext());
                static_cast<void>(cancelResult);
                GASSERT(cancelResult);
            }
        }
        else  {
            GASSERT(op == OpType::Write);
            if (iter->writeLength_ == 0) {
                return false;
            }
            iter->writeLength_ = 0;
            if (currentOp) {
                auto cancelResult = device_.cancelWrite(EventLoopContext());
                static_cast<void>(cancelResult);
                GASSERT(cancelResult);
            }
        }

        bool erase = ((iter->readLength_ == 0) && (iter->writeLength_ == 0));
        if (!erase) {
            return true;
        }

        opQueue_.erase(iter);
        if (!currentOp) {
            return true;
        }

        // Cancelled current op, new one must be rescheduled
        guard.release();
        startNextOpIfAvailable(EventLoopContext());
        return true;
    }

    void canDoInterruptHandler(OpType op)
    {
        GASSERT(!opQueue_.empty());
        auto iter = findDeviceInfo(opQueue_.front().id_);
        if (iter == infos_.end()) {
            GASSERT(!"Mustn't happen");
            return;
        }

        if ((op == OpType::Read) && (iter->canReadHandler_)) {
            iter->canReadHandler_();
        }
        else if ((op == OpType::Write) && (iter->canWriteHandler_)) {
            iter->canWriteHandler_();
        }
    }

    void opCompleteInterruptHandler(const embxx::error::ErrorStatus& es, OpType op)
    {
        GASSERT(!opQueue_.empty());
        auto& opInfo = opQueue_.front();
        auto iter = findDeviceInfo(opInfo.id_);
        if (iter == infos_.end()) {
            GASSERT(!"Mustn't happen");
            return;
        }

        if (op == OpType::Read) {
            GASSERT(opInfo.readLength_ != 0);
            opInfo.readLength_ = 0;
        }
        else if (op == OpType::Write) {
            GASSERT(opInfo.writeLength_ != 0);
            opInfo.writeLength_ = 0;
        }

        if ((opInfo.readLength_ == 0) && (opInfo.writeLength_ == 0)) {
            opQueue_.popFront();
            startNextOpIfAvailable(InterruptContext());
        }

        if ((op == OpType::Read) && (iter->readCompleteHandler_)) {
            iter->readCompleteHandler_(es);
        }
        else if ((op == OpType::Write) && (iter->writeCompleteHandler_)) {
            iter->writeCompleteHandler_(es);
        }
    }

    bool suspendDeviceEventLoopCtx()
    {
        if (!suspended_) {
            suspended_ = device_.suspend(EventLoopContext());
            return suspended_;
        }
        return false;
    }

    void resumeDeviceEventLoopCtx()
    {
        GASSERT(suspended_);
        suspended_ = false;
        device_.resume(EventLoopContext());
    }

    Device& device_;
    DeviceInfosArray infos_;
    OpQueue opQueue_;
    bool suspended_;
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

    explicit DeviceOpQueue(Device& dev)
      : device_(dev)
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

