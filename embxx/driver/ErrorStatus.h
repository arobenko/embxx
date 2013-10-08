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

/// @file embxx/driver/ErrorStatus.h
/// This file contains definition of common error status for all the drivers in
/// "driver" module.

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



