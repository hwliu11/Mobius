//-----------------------------------------------------------------------------
// Created on: 15 September 2014
//-----------------------------------------------------------------------------
// Copyright (c) 2017, Sergey Slyadnev
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//    * Neither the name of Sergey Slyadnev nor the
//      names of all contributors may be used to endorse or promote products
//      derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//-----------------------------------------------------------------------------

// Own include
#include <mobius/core_JournalTicket.h>

//-----------------------------------------------------------------------------
// Journal Ticket
//-----------------------------------------------------------------------------

//! Default constructor.
mobius::core_JournalTicket::core_JournalTicket()
{
  this->init("");
}

//! Constructor.
//! \param msg [in] message.
mobius::core_JournalTicket::core_JournalTicket(const char* msg)
{
  this->init(msg);
}

//! Constructor.
//! \param msg [in] message.
mobius::core_JournalTicket::core_JournalTicket(const std::string& msg)
{
  this->init( msg.c_str() );
}

//! Constructor.
//! \param buff [in] message buffer.
mobius::core_JournalTicket::core_JournalTicket(const core_StringBuffer& buff)
{
  this->init( buff.String.c_str() );
}

//! Destructor.
mobius::core_JournalTicket::~core_JournalTicket()
{}

//! Converts the ticket to string.
//! \return string representation of a ticket.
std::string mobius::core_JournalTicket::ToString() const
{
  return this->Msg;
}

//! Initializes the ticket with message.
//! \param msg [in] message to set.
void mobius::core_JournalTicket::init(const char* msg)
{
  this->Msg = msg;
}
