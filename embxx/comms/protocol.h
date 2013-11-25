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

/// @file embxx/comms/protocol.h
/// Common file to include all supported protocol layers.

#pragma once

#include "protocol/MsgDataLayer.h"
#include "protocol/MsgIdLayer.h"
#include "protocol/MsgSizeLayer.h"
#include "protocol/ChecksumLayer.h"
#include "protocol/SyncPrefixLayer.h"
