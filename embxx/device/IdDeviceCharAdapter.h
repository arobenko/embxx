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

namespace embxx
{

namespace device
{

/// @ingroup device
/// @brief Wrapper to embxx::device::DevicoOpQueue object that exposed interface
///        required by embxx::driver::Character driver.
/// @details The purpose of this class is to adapt the interface of
///          embxx::device::DeviceOpQueue to the one required by
///          embxx::driver::Character and to be able to use the "character"
///          driver to read/write data from/to device that require an
///          identification of the target/source entity prior to executing the
///          operation.
/// @tparam TDevice Device interface of which must be adapted. The class was
///         written in intention to adapt interface of embxx::device::DeviceOpQueue.
///         If the latter is not used the provided device must expose the same
///         interface.
/// @headerfile embxx/device/IdDeviceCharAdapter.h
template <typename TDevice>
class IdDeviceCharAdapter
{
public:
    /// @brief Type of the underlaying device.
    typedef TDevice Device;

    /// @brief Character type defined in the wrapped device
    typedef typename TDevice::CharType CharType;

    /// @brief Device identification type defined in the wrapped device class.
    typedef typename TDevice::DeviceIdType DeviceIdType;

    /// @brief constructor
    /// @param dev Reference to the wrapped device
    /// @param idVal ID of the device to be used for all the issued operations.
    IdDeviceCharAdapter(Device& dev, DeviceIdType idVal)
      : device_(dev),
        id_(idVal)
    {
    }

    /// @brief Copy constructor is default
    IdDeviceCharAdapter(const IdDeviceCharAdapter&) = default;

    /// @brief Destructor is default
    ~IdDeviceCharAdapter() = default;

    /// @brief Copy assignement is deleted.
    IdDeviceCharAdapter& operator=(const IdDeviceCharAdapter&) = delete;

    /// @brief Access to the underlying device.
    Device& device()
    {
        return device_;
    }

    /// @brief Const version of the device().
    const Device& device() const
    {
        return device_;
    }

    /// @brief Get device ID.
    const DeviceIdType id() const
    {
        return id_;
    }

    /// @brief Set "can read" handler.
    /// @details Calls setCanReadHandler() of the underlying device with
    ///          stored ID information.
    template <typename TFunc>
    void setCanReadHandler(TFunc&& func)
    {
        device_.setCanReadHandler(id_, std::forward<TFunc>(func));
    }

    /// @brief Set "can write" handler.
    /// @details Calls setCanWriteHandler() of the underlying device with
    ///          stored ID information.
    template <typename TFunc>
    void setCanWriteHandler(TFunc&& func)
    {
        device_.setCanWriteHandler(id_, std::forward<TFunc>(func));
    }

    /// @brief Set "read complete" handler.
    /// @details Calls setReadCompleteHandler() of the underlying device with
    ///          stored ID information.
    template <typename TFunc>
    void setReadCompleteHandler(TFunc&& func)
    {
        device_.setReadCompleteHandler(id_, std::forward<TFunc>(func));
    }

    /// @brief Set "write complete" handler.
    /// @details Calls setWriteCompleteHandler() of the underlying device with
    ///          stored ID information.
    template <typename TFunc>
    void setWriteCompleteHandler(TFunc&& func)
    {
        device_.setWriteCompleteHandler(id_, std::forward<TFunc>(func));
    }

    /// @brief Forwards startRead() request to the underlying device.
    /// @details Adds stored ID information to the parameters.
    template <typename... TArgs>
    void startRead(TArgs&&... args)
    {
        device_.startRead(id_, std::forward<TArgs>(args)...);
    }

    /// @brief Forwards cancelRead() request to the underlying device.
    /// @details Adds stored ID information to the parameters.
    template <typename... TArgs>
    bool cancelRead(TArgs&&... args)
    {
        return device_.cancelRead(id_, std::forward<TArgs>(args)...);
    }

    /// @brief Forwards startWrite() request to the underlying device.
    /// @details Adds stored ID information to the parameters.
    template <typename... TArgs>
    void startWrite(TArgs&&... args)
    {
        device_.startWrite(id_, std::forward<TArgs>(args)...);
    }

    /// @brief Forwards cancelWrite() request to the underlying device.
    /// @details Adds stored ID information to the parameters.
    template <typename... TArgs>
    bool cancelWrite(TArgs&&... args)
    {
        return device_.cancelWrite(id_, std::forward<TArgs>(args)...);
    }

    /// @brief Forwards suspend() request to the underlying device.
    /// @details Adds stored ID information to the parameters.
    template <typename... TArgs>
    bool suspend(TArgs&&... args)
    {
        return device_.suspend(id_, std::forward<TArgs>(args)...);
    }

    /// @brief Forwards resume() request to the underlying device.
    /// @details Adds stored ID information to the parameters.
    template <typename... TArgs>
    void resume(TArgs&&... args)
    {
        device_.resume(id_, std::forward<TArgs>(args)...);
    }

    /// @brief Forwards canRead() call to the underlying device.
    template <typename... TArgs>
    bool canRead(TArgs&&... args)
    {
        return device_.canRead(id_, std::forward<TArgs>(args)...);
    }

    /// @brief Forwards canWrite() call to the underlying device.
    template <typename... TArgs>
    bool canWrite(TArgs&&... args)
    {
        return device_.canWrite(id_, std::forward<TArgs>(args)...);
    }

    /// @brief Forwards read() call to the underlying device.
    template <typename... TArgs>
    CharType read(TArgs&&... args)
    {
        return device_.read(id_, std::forward<TArgs>(args)...);
    }

    /// @brief Forwards write() call to the underlying device.
    template <typename... TArgs>
    void write(TArgs&&... args)
    {
        device_.write(id_, std::forward<TArgs>(args)...);
    }

private:
    Device& device_;
    DeviceIdType id_;
};

}  // namespace device

}  // namespace embxx

