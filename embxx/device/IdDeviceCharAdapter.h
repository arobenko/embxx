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
    /// @param device Reference to the wrapped device
    /// @param id ID of the device to be used for all the issued operations.
    IdDeviceCharAdapter(Device& device, DeviceIdType id);

    /// @brief Copy constructor is default
    IdDeviceCharAdapter(const IdDeviceCharAdapter&) = default;

    /// @brief Destructor is default
    ~IdDeviceCharAdapter() = default;

    /// @brief Copy assignement is deleted.
    IdDeviceCharAdapter& operator=(const IdDeviceCharAdapter&) = delete;

    /// @brief Access to the underlying device.
    Device& device();

    /// @brief Const version of the device().
    const Device& device() const;

    /// @brief Set "can read" handler.
    /// @details Calls setCanReadHandler() of the underlying device with
    ///          stored ID information.
    template <typename TFunc>
    void setCanReadHandler(TFunc&& func);

    /// @brief Set "can write" handler.
    /// @details Calls setCanWriteHandler() of the underlying device with
    ///          stored ID information.
    template <typename TFunc>
    void setCanWriteHandler(TFunc&& func);

    /// @brief Set "read complete" handler.
    /// @details Calls setReadCompleteHandler() of the underlying device with
    ///          stored ID information.
    template <typename TFunc>
    void setReadCompleteHandler(TFunc&& func);

    /// @brief Set "write complete" handler.
    /// @details Calls setWriteCompleteHandler() of the underlying device with
    ///          stored ID information.
    template <typename TFunc>
    void setWriteCompleteHandler(TFunc&& func);

    /// @brief Forwards startRead() request to the underlying device.
    /// @details Adds stored ID information to the parameters.
    template <typename... TArgs>
    void startRead(TArgs&&... args);

    /// @brief Forwards cancelRead() request to the underlying device.
    /// @details Adds stored ID information to the parameters.
    template <typename... TArgs>
    bool cancelRead(TArgs&&... args);

    /// @brief Forwards startWrite() request to the underlying device.
    /// @details Adds stored ID information to the parameters.
    template <typename... TArgs>
    void startWrite(TArgs&&... args);

    /// @brief Forwards cancelWrite() request to the underlying device.
    /// @details Adds stored ID information to the parameters.
    template <typename... TArgs>
    bool cancelWrite(TArgs&&... args);

    /// @brief Forwards canRead() call to the underlying device.
    template <typename... TArgs>
    bool canRead(TArgs&&... args);

    /// @brief Forwards canWrite() call to the underlying device.
    template <typename... TArgs>
    bool canWrite(TArgs&&... args);

    /// @brief Forwards read() call to the underlying device.
    template <typename... TArgs>
    CharType read(TArgs&&... args);

    /// @brief Forwards write() call to the underlying device.
    template <typename... TArgs>
    void write(TArgs&&... args);

private:
    Device& device_;
    DeviceIdType id_;
};

// Implementation
template <typename TDevice>
IdDeviceCharAdapter<TDevice>::IdDeviceCharAdapter(
    Device& device,
    DeviceIdType id)
    : device_(device),
      id_(id)
{
}

template <typename TDevice>
typename IdDeviceCharAdapter<TDevice>::Device&
IdDeviceCharAdapter<TDevice>::device()
{
    return device_;
}

template <typename TDevice>
const typename IdDeviceCharAdapter<TDevice>::Device&
IdDeviceCharAdapter<TDevice>::device() const
{
    return device_;
}

template <typename TDevice>
template <typename TFunc>
void IdDeviceCharAdapter<TDevice>::setCanReadHandler(TFunc&& func)
{
    device_.setCanReadHandler(id_, std::forward<TFunc>(func));
}

template <typename TDevice>
template <typename TFunc>
void IdDeviceCharAdapter<TDevice>::setCanWriteHandler(TFunc&& func)
{
    device_.setCanWriteHandler(id_, std::forward<TFunc>(func));
}

template <typename TDevice>
template <typename TFunc>
void IdDeviceCharAdapter<TDevice>::setReadCompleteHandler(TFunc&& func)
{
    device_.setReadCompleteHandler(id_, std::forward<TFunc>(func));
}

template <typename TDevice>
template <typename TFunc>
void IdDeviceCharAdapter<TDevice>::setWriteCompleteHandler(TFunc&& func)
{
    device_.setWriteCompleteHandler(id_, std::forward<TFunc>(func));
}

template <typename TDevice>
template <typename... TArgs>
void IdDeviceCharAdapter<TDevice>::startRead(TArgs&&... args)
{
    device_.startRead(id_, std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
bool IdDeviceCharAdapter<TDevice>::cancelRead(TArgs&&... args)
{
    return device_.cancelRead(id_, std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
void IdDeviceCharAdapter<TDevice>::startWrite(TArgs&&... args)
{
    device_.startWrite(id_, std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
bool IdDeviceCharAdapter<TDevice>::cancelWrite(TArgs&&... args)
{
    return device_.cancelWrite(id_, std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
bool IdDeviceCharAdapter<TDevice>::canRead(TArgs&&... args)
{
    return device_.canRead(std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
bool IdDeviceCharAdapter<TDevice>::canWrite(TArgs&&... args)
{
    return device_.canWrite(std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
typename IdDeviceCharAdapter<TDevice>::CharType
IdDeviceCharAdapter<TDevice>::read(TArgs&&... args)
{
    return device_.read(std::forward<TArgs>(args)...);
}

template <typename TDevice>
template <typename... TArgs>
void IdDeviceCharAdapter<TDevice>::write(TArgs&&... args)
{
    device_.write(std::forward<TArgs>(args)...);
}


}  // namespace device

}  // namespace embxx

