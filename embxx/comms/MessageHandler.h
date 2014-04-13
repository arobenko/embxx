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

/// @file embxx/comms/MessageHandler.h
/// This file contains definition of common message handler.

#pragma once

#include <tuple>
#include <type_traits>

#include "embxx/util/Tuple.h"
#include "Message.h"

namespace embxx
{

namespace comms
{

/// @ingroup comms
/// @brief Generic common message handler.
/// @details Will define virtual message handling functions for all the
///          messages bundled in TAllMessages plus one to handle TMsgBase
///          type of message as default behaviour. The declaration of the
///          handling function is as following:
///          @code
///          virtual void handleMessage(ActualMessageType& msg);
///          @endcode
///          All the handling functions will upcast the message to TMsgBase and
///          call the default message handling function with signature:
///          @code
///          virtual void handleMessage(TMsgBase& msg);
///          @endcode
///          which does nothing. To override the handling behaviour just inherit
///          your handler from comms::MessageHandler and override the appropriate
///          function.
/// @tparam TMsgBase Base class of all custom messages bundled in TAllMessages.
/// @tparam TAllMessages All message types bundled in std::tuple that need to
///         be handled.
/// @pre TAllMessages is any variation of std::tuple
/// @pre TMsgBase is a base class for all the custom messages in TAllMessages.
/// @headerfile embxx/comms/MessageHandler.h
template <typename TMsgBase, typename TAllMessages>
class MessageHandler
{
    static_assert(util::IsTuple<TAllMessages>::Value,
                  "TAllMessages must be std::tuple");
};

/// @cond DOCUMENT_MESSAGE_HANDLER_SPECIALISATION
template <typename TMsgBase, typename TFirst, typename... TRest>
class MessageHandler<TMsgBase, std::tuple<TFirst, TRest...> > :
                        public MessageHandler<TMsgBase, std::tuple<TRest...> >
{
    typedef MessageHandler<TMsgBase, std::tuple<TRest...> > Base;
public:

    virtual ~MessageHandler() {}

    using Base::handleMessage;
    virtual void handleMessage(TFirst& msg)
    {
        static_assert(std::is_base_of<TMsgBase, TFirst>::value,
            "TMsgBase must be base class for all messages");

        this->handleMessage(static_cast<TMsgBase&>(msg));
    }
};

template <typename TMsgBase>
class MessageHandler<TMsgBase, std::tuple<> >
{
public:

    virtual ~MessageHandler() {}

    virtual void handleMessage(TMsgBase& msg)
    {
        // Nothing to do
        static_cast<void>(msg);
    }
};

/// @endcond

}  // namespace comms

}  // namespace embxx
