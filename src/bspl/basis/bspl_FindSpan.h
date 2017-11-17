//-----------------------------------------------------------------------------
// Created on: 11 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

#ifndef bspl_FindSpan_HeaderFile
#define bspl_FindSpan_HeaderFile

// bspl includes
#include <mobius/bspl.h>

namespace mobius {

//! \ingroup MOBIUS_BSPL
//!
//! Algorithm finding knot span index.
//!
//! Inputs:
//!   u -- target knot;
//!   U -- knot vector;
//!   p -- degree;
//!
//! Outputs:
//!   index of the knot span;
//!
//! \todo complete description
class bspl_FindSpan
{
public:

  mobiusBSpl_EXPORT
    bspl_FindSpan(const std::vector<double>& U,
                  const int                  p);

public:

  mobiusBSpl_EXPORT int
    operator()(const double u) const;

protected:

  std::vector<double> m_U;    //!< Knot vector.
  int                 m_iDeg; //!< Degree.

};

};

#endif
