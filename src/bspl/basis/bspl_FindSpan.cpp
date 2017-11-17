//-----------------------------------------------------------------------------
// Created on: 11 June 2013
// Created by: Sergey SLYADNEV
//-----------------------------------------------------------------------------

// bspl includes
#include <mobius/bspl_FindSpan.h>

#undef COUT_DEBUG
#if defined COUT_DEBUG
  #pragma message("===== warning: COUT_DEBUG is enabled")
#endif

//-----------------------------------------------------------------------------

//! Initializes the tool with all necessary B-spline properties.
//! \param U [in] knot vector.
//! \param p [in] degree.
//! \return span index.
mobius::bspl_FindSpan::bspl_FindSpan(const std::vector<double>& U,
                                     const int                  p)
{
  m_U    = U;
  m_iDeg = p;
}

//-----------------------------------------------------------------------------

//! Finds the target span index by binary search.
//! \param u [in] target parameter.
//! \return span index.
int mobius::bspl_FindSpan::operator()(const double u) const
{
  const int nU = (int) m_U.size();
  const int p  = m_iDeg;
  const int m  = nU - 1;
  const int n  = m - p - 1;
  //
  if ( u == m_U[n + 1] )
    return n;

  int  mid_idx;
  int  min_idx = 0;
  int  max_idx = nU;
  bool isFound = false;

#if defined COUT_DEBUG
  std::cout << "\tu = " << u << std::endl;
#endif

  do
  {
    mid_idx = (min_idx + max_idx) / 2;

    if ( mid_idx == min_idx || mid_idx == max_idx )
    {
      isFound = true;
      break;
    }

    const double mid_u = m_U[mid_idx];

#if defined COUT_DEBUG
    std::cout << "\tmid_u = " << mid_u << std::endl;
#endif

    if ( mid_u <= u )
      min_idx = mid_idx;
    else if ( mid_u > u )
      max_idx = mid_idx;
  }
  while ( !isFound );

  return mid_idx;
}
