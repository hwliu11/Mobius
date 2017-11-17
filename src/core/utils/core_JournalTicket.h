//-----------------------------------------------------------------------------
// Created on: 15 September 2014
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------
// This file is a part of Mobius library. It has no license protection. You
// are free to use it the way you like, including incorporating of Mobius
// sources and libraries into commercial applications.
// Web: http://Mobius.su
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
    core::split(ticket_1.Msg, " \t", tokens_1);
    core::split(ticket_2.Msg, " \t", tokens_2);

    do
    {
      token_1 = ( t1 >= tokens_1.size() || tokens_1.empty() ) ? "" : tokens_1[t1++];
      token_2 = ( t2 >= tokens_2.size() || tokens_2.empty() ) ? "" : tokens_2[t2++];

      bool areEqual;
      if ( core::is_number(token_1) && core::is_number(token_2) )
      {
        val1 = core::to_number<double>(token_1, 0.0);
        val2 = core::to_number<double>(token_2, 0.0);

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
        areEqual = core::are_equal(token_1, token_2);

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
