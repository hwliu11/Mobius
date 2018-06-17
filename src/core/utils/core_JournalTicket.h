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

#ifndef core_JournalTicket_HeaderFile
#define core_JournalTicket_HeaderFile

// core includes
#include <mobius/core_StringBuffer.h>

namespace mobius {

//! \ingroup MOBIUS_CORE
//!
//! Auxiliary structure representing a single journaled record.
class core_JournalTicket
{
public:

  mobiusCore_EXPORT
    core_JournalTicket();

  mobiusCore_EXPORT
    core_JournalTicket(const char* msg);

  mobiusCore_EXPORT
    core_JournalTicket(const std::string& msg);

  mobiusCore_EXPORT
    core_JournalTicket(const core_StringBuffer& buff);

  mobiusCore_EXPORT
    ~core_JournalTicket();

public:

  mobiusCore_EXPORT virtual std::string
    ToString() const;

protected:

  void init(const char* msg);

public:

  std::string Msg; //!< Ticket message.

};

//! \ingroup MOBIUS_CORE
//!
//! Comparator for Tickets with intelligent treatment of floating point
//! values.
class core_FltTicketCompare
{
public:

  //! Constructs comparator.
  //! \param resolution [in] resolution for small values.
  //! \param precision  [in] comparison precision for values passing
  //!                        resolution test.
  core_FltTicketCompare(const double resolution,
                        const double precision)
  : m_fResolution(resolution),
    m_fPrecision(precision)
  {}

public:

  //! Comparison routine.
  //! \param ticket_1 [in] first Ticket to compare.
  //! \param ticket_2 [in] second Ticket to compare.
  //! \return true if Tickets are equal, false -- otherwise.
  bool operator()(const core_JournalTicket& ticket_1,
                  const core_JournalTicket& ticket_2) const
  {
    std::string token_1, token_2;
    size_t t1 = 0, t2 = 0;
    double val1, val2, actDev;

    std::vector<std::string> tokens_1, tokens_2;
    tokens_1.reserve(512); tokens_2.reserve(512);
    core::str::split(ticket_1.Msg, " \t", tokens_1);
    core::str::split(ticket_2.Msg, " \t", tokens_2);

    do
    {
      token_1 = ( t1 >= tokens_1.size() || tokens_1.empty() ) ? "" : tokens_1[t1++];
      token_2 = ( t2 >= tokens_2.size() || tokens_2.empty() ) ? "" : tokens_2[t2++];

      bool areEqual;
      if ( core::str::is_number(token_1) && core::str::is_number(token_2) )
      {
        val1 = core::str::to_number<double>(token_1, 0.0);
        val2 = core::str::to_number<double>(token_2, 0.0);

        if ( fabs(val1 - val2) < m_fResolution )
          areEqual = true;
        else
        {
          const double avg = (val1 + val2)/2.0;
          actDev = fabs( (val1 - val2)/avg )*100.0;
          areEqual = ( actDev < m_fPrecision );
        }
      }
      else
        areEqual = core::str::are_equal(token_1, token_2);

      if ( !areEqual )
        return false;
    }
    while ( token_1.size() && token_2.size() );

    // If we are here, then every token has passed the check
    return true;
  }

private:

  double m_fResolution; //!< Resolution.
  double m_fPrecision;  //!< Precision.

};

//! \ingroup MOBIUS_CORE
//!
//! Convenience type alias.
typedef core_JournalTicket ticket;

};

#endif
