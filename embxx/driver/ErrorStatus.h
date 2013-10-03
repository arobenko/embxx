//
// Copyright 2013 (C). Alex Robenko. All rights reserved.
//

#pragma once

namespace embxx
{

namespace driver
{

/// @ingroup driver
/// @brief Common error status of driver operation
/// @details Reported to the asynchronous operation handler.
enum class ErrorStatus
{
    Success, ///< Successful completion of operation.
    Aborted, ///< The operation was cancelled/aborted.
    InternalError, ///< Unexpected internal error occurred
    NumOfStatuses ///< Number of available statuses. Must be last
};

}  // namespace driver

}  // namespace embxx



