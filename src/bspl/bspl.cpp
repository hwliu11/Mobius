//-----------------------------------------------------------------------------
// Created on: 19 July 2017
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// BSpl includes
#include <mobius/bspl.h>

//-----------------------------------------------------------------------------

int mobius::bspl::M(const int n, const int p)
{
  return n + p + 1; // Common formula
}

//-----------------------------------------------------------------------------

int mobius::bspl::NumberOfKnots(const int n, const int p)
{
  return M(n, p) + 1;
}

//-----------------------------------------------------------------------------

bool mobius::bspl::Check(const int n, const int p)
{
  const int r = NumberOfKnots(n, p);
  if ( r < 2*(p + 1) )
    return false;

  return true;
}
