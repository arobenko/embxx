//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

/// @file comms/ErrorStatus.h
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
