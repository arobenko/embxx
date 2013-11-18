//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

// This library is free software: you can redistribute it and/or modify
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

/// @file embxx/comms/ErrorStatus.h
/// This file contain definition of error statuses used by comms module.

#pragma once

namespace embxx
{

namespace comms
{

/// @ingroup comms
/// @brief Error statuses reported by the Communication module.
/// @headerfile embxx/comms/ErrorStatus.h "embxx/comms/ErrorStatus.h"
enum class ErrorStatus {
    Success, ///< Used to indicate successful outcome of the operation.
    UpdateRequired, ///< Used to indicate that write operation wasn't complete,
                    /// call to update(...) is required.
    NotEnoughData, ///< Used to indicate that stream buffer didn't contain
                   /// enough data to complete read operation.
    ProtocolError, ///< Used to indicate that any of the used protocols
                   /// encountered an error while processing the data.
    BufferOverflow, ///< Used to indicate that stream buffer was overflowed
                    /// when attempting to write data.
    InvalidMsgId, ///< Used to indicate that received message has unknown id
    InvalidMsgData, ///<Used to indicate that received message has invalid
                    /// data.
    MsgAllocFaulure, ///<Used to indicate that message allocation has failed.
    NumOfErrorStatuses ///< Number of supported error statuses, must be last.
};

}  // namespace comms

}  // namespace embxx
