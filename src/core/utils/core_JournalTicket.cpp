//-----------------------------------------------------------------------------
// Created on: 15 September 2014
// Created by: Sergey SLYADNEV
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
