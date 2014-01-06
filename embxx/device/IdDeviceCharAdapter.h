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

template <typename TDevice>
class IdDeviceCharAdapter
{
public:
    typedef TDevice Device;
    typedef typename TDevice::CharType CharType;
    typedef typename TDevice::DeviceIdType DeviceIdType;

    IdDeviceCharAdapter(Device& device, DeviceIdType addr);
    IdDeviceCharAdapter(const IdDeviceCharAdapter&) = default;
    ~IdDeviceCharAdapter() = default;

    IdDeviceCharAdapter& operator=(const IdDeviceCharAdapter&) = delete;

    Device& device();
    const Device& device() const;

    template <typename TFunc>
    void setCanReadHandler(TFunc&& func);

    template <typename TFunc>
    void setCanWriteHandler(TFunc&& func);

    template <typename TFunc>
    void setReadCompleteHandler(TFunc&& func);

    template <typename TFunc>
    void setWriteCompleteHandler(TFunc&& func);

    template <typename... TArgs>
    void startRead(TArgs&&... args);

    template <typename... TArgs>
    bool cancelRead(TArgs&&... args);

    template <typename... TArgs>
    void startWrite(TArgs&&... args);

    template <typename... TArgs>
    bool cancelWrite(TArgs&&... args);

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

